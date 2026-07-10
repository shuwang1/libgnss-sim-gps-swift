/******************************************************************************
 *
 * Aut viam inveniam aut faciam
 *
 * Copyright (c) 2019-2026 Shu Wang. All rights reserved.
 *
 * PROPRIETARY AND CONFIDENTIAL
 *
 * This software and its documentation (the "Software") are the confidential 
 * and proprietary information of Shu Wang. All rights, title, and 
 * interest in and to the Software, including all intellectual property rights, 
 * are and shall remain the exclusive property of Shu Wang.
 *
 * Correspondence regarding this Software should be directed to:
 * Shu Wang <shuwang1@outlook.com>
 ******************************************************************************/

import Foundation

/// Main simulation engine for GNSS signal generation.
/// 
/// The `Simulator` class orchestrates the entire simulation process, from loading RINEX data and trajectory files
/// to managing hardware channels and generating baseband I/Q samples.
class Simulator {
    
    /// Configuration parameters for the simulation run.
    struct Config {
        /// Path to the RINEX navigation file.
        var navFile: String = ""
        /// Path to the user motion file (CSV or NMEA).
        var umFile: String = ""
        /// Path to the output binary signal file.
        var outFile: String = ""
        /// Sampling frequency in Hz (default: 2.6 MHz).
        var sampFreq: Double = 2600000.0
        /// Elevation mask in degrees to filter satellites below the horizon.
        var elvMask: Double = 5.0
        /// Bits per sample (1, 8, or 16).
        var dataFormat: Int = 16
        /// Maximum simulation duration in seconds.
        var duration: Int = 300
        /// Initial GPS time. If week is -1, it's automatically determined from RINEX.
        var g0: GPSTime = GPSTime(week: -1, sec: 0)
        /// If true, the receiver position remains fixed at the first point.
        var staticMode: Bool = false
        /// If true, trajectory file is interpreted as Latitude, Longitude, Height.
        var umLLH: Bool = false
        /// If true, trajectory file is interpreted as NMEA GGA stream.
        var nmeaGGA: Bool = false
        /// Enables verbose debug output.
        var verbose: Bool = false
    }
    
    /// Parsed ionospheric and UTC parameters.
    var ionUTC: IonUTC = IonUTC()
    /// Current configuration.
    var config: Config
    /// Receiver trajectory as a list of ECEF positions.
    var xyz: [Vector3] = []
    /// 2D array of ephemerides (epochs x satellites).
    var eph: [[Ephemeris]] = []
    /// Index of the current ephemeris epoch in use.
    var ieph: Int = 0
    /// Current receiver clock time in GPS time.
    var receiverTime: GPSTime = GPSTime(week: 0, sec: 0)
    /// Simulated hardware channels.
    var channels: [Link] = [Link](repeating: Link(), count: Constants.MAX_CHAN)
    /// Mapping from PRN index to channel index (-1 if not tracked).
    var allocatedSat: [Int] = [Int](repeating: -1, count: Constants.MAX_SAT)
    /// Interpolated antenna gain pattern.
    var antPat: [Double] = []
    /// Time between samples (1 / sampFreq).
    var samplePeriod: Double = 0
    /// Total number of 0.1s simulation steps to execute.
    var numSteps: Int = 0
    
    // Pre-allocated buffers for synthesis
    var iAccBuff: [Int] = []
    var qAccBuff: [Int] = []

    /// Initializes a new simulator with the provided configuration.
    /// - Parameter config: Simulation configuration.
    init(config: Config) {
        self.config = config
        self.antPat = LUT.ant_pat_db.map { pow(10.0, -$0 / 20.0) }
        self.samplePeriod = 1.0 / config.sampFreq
    }
    
    /// Initializes the simulation environment by loading files and determining the starting epoch.
    /// - Returns: `true` if initialization succeeded, `false` otherwise.
    func initialize() -> Bool {
        if config.staticMode {
            if xyz.isEmpty {
                // Default to Erlang Network location if none provided (as in C code)
                let defaultLLH = Vector3(52.2 * Constants.D2R, 0.1 * Constants.D2R, 100.0)
                xyz = [MathUtils.llh2xyz(defaultLLH)]
                Logger.info("Static mode: Using default location (52.2N, 0.1E)")
            } else {
                Logger.info("Static mode: Using initial trajectory point")
            }
        } else {
            if config.nmeaGGA {
                if let path = Trajectory.readNmeaGGA(filename: config.umFile) { xyz = path }
            } else if config.umLLH {
                if let path = Trajectory.readUserMotionLLH(filename: config.umFile) { xyz = path }
            } else {
                if let path = Trajectory.readUserMotion(filename: config.umFile) { xyz = path }
            }
        }
        if xyz.isEmpty { return false }
        Logger.info("Loaded \(xyz.count) trajectory points. First point: \(xyz[0].x), \(xyz[0].y), \(xyz[0].z)")
        
        if config.staticMode || xyz.count == 1 {
            numSteps = config.duration * 10
        } else {
            numSteps = xyz.count
        }
        
        if numSteps > config.duration * 10 { numSteps = config.duration * 10 }
        
        if let result = GPSEphemeris.loadGPSEphemeris(fname: config.navFile) {
            self.eph = result.epochs
            self.ionUTC = result.ionUTC
        } else {
            return false
        }
        
        if config.g0.week == -1 {
            for sv in 0..<Constants.MAX_SAT {
                if eph[0][sv].vflg {
                    config.g0 = eph[0][sv].toc
                    break
                }
            }
        }
        
        if config.g0.week == -1 { return false }
        
        self.ieph = -1
        for i in 0..<eph.count {
            for sv in 0..<Constants.MAX_SAT {
                if eph[i][sv].vflg {
                    if abs(config.g0 - eph[i][sv].toc) < 3600.0 {
                        self.ieph = i
                        break
                    }
                }
            }
            if ieph >= 0 { break }
        }
        
        if ieph == -1 { return false }
        
        receiverTime = config.g0
        allocateChannels()

        let iqBuffSize = Int(config.sampFreq * Constants.TIME_STEP)
        iAccBuff = [Int](repeating: 0, count: iqBuffSize)
        qAccBuff = [Int](repeating: 0, count: iqBuffSize)

        let visibleCount = allocatedSat.filter { $0 != -1 }.count
        Logger.info("Initialization complete. \(visibleCount) satellites visible.")
        return true
    }
    
    /// Dynamically allocates channels to visible satellites.
    func allocateChannels() {
        let currentXYZ = config.staticMode ? (xyz.first ?? Vector3(0,0,0)) : xyz[0]

        // PERFORMANCE OPTIMIZATION: Precompute LLH and TMat for the receiver
        // to avoid redundant iterative trigonometry calculations inside the satellite loop.
        // Impact: Reduces CPU overhead per time step significantly.
        let currentLLH = MathUtils.xyz2llh(currentXYZ)
        let currentTMat = MathUtils.ltcmat(currentLLH)

        for sv in 0..<Constants.MAX_SAT {
            var azel: (az: Double, el: Double) = (0, 0)
            let visible = checkSatVisibility(sv: sv, g: receiverTime, xyz: currentXYZ, elvMask: config.elvMask, azel: &azel, llh: currentLLH, tmat: currentTMat)
            
            if visible == 1 {
                if allocatedSat[sv] == -1 {
                    for i in 0..<Constants.MAX_CHAN {
                        if channels[i].prn == 0 {
                            var link = Link()
                            link.prn = sv + 1
                            link.sys = .gps
                            link.mod = .bpsk
                            link.codeLength = 1023
                            link.azel = azel
                            if let code = GPSCode.generateL1CA(prn: link.prn) {
                                link.ca = code
                            }
                            
                            link.sbf = Link.eph2sbf(eph: eph[ieph][sv], ionoutc: ionUTC)
                            link.generateNavMsg(g: receiverTime, initFlag: true)
                            
                            let satData = Positioning.calculateSatPos(eph: eph[ieph][sv], g: receiverTime)
                            let rho = Channel.estimateRange(ionoutc: ionUTC, g: receiverTime, xyz: currentXYZ, satPos: satData.pos, satVel: satData.vel, clk: satData.clk, llh: currentLLH, tmat: currentTMat)
                            
                            link.rho0 = rho
                            link.carrPhase = 0
                            channels[i] = link
                            allocatedSat[sv] = i
                            Logger.debug("Allocated Channel \(i) for PRN \(link.prn) (Az: \(String(format: "%.1f", azel.az * Constants.R2D)), El: \(String(format: "%.1f", azel.el * Constants.R2D)))")
                            break
                        }
                    }
                }
            } else if allocatedSat[sv] >= 0 {
                channels[allocatedSat[sv]].prn = 0
                allocatedSat[sv] = -1
            }
        }
    }
    
    /// Checks if a satellite is visible above the elevation mask.
    /// - Parameters:
    ///   - sv: PRN index (0-31).
    ///   - g: Target time.
    ///   - xyz: Receiver position.
    ///   - elvMask: Mask angle in degrees.
    ///   - azel: Output Azimuth/Elevation.
    ///   - llh: Precomputed receiver LLH position.
    ///   - tmat: Precomputed receiver local tangent plane matrix.
    /// - Returns: 1 if visible, 0 if below mask, -1 if ephemeris invalid.
    func checkSatVisibility(sv: Int, g: GPSTime, xyz: Vector3, elvMask: Double, azel: inout (az: Double, el: Double), llh: Vector3, tmat: [[Double]]) -> Int {
        let e = eph[ieph][sv]
        if !e.vflg { return -1 }
        
        let satData = Positioning.calculateSatPos(eph: e, g: g)
        let los = satData.pos - xyz
        let neu = MathUtils.ecef2neu(los, t: tmat)
        azel = MathUtils.neu2azel(neu)
        
        let elDeg = azel.el * Constants.R2D
        return (elDeg > elvMask) ? 1 : 0
    }
    
    /// Executes one 0.1s simulation step.
    /// - Parameter stepIdx: Index of the current step.
    /// - Returns: A buffer of interleaved Int16 I/Q samples, or `nil` if simulation finished.
    func step(stepIdx: Int) -> [Int16]? {
        if stepIdx >= numSteps { return nil }
        
        Logger.simTime = Double(stepIdx) / 10.0
        
        let currentXYZ: Vector3
        if config.staticMode || xyz.count == 1 {
            currentXYZ = xyz[0]
        } else {
            currentXYZ = xyz[stepIdx]
        }

        // PERFORMANCE OPTIMIZATION: Precompute LLH and TMat for the receiver
        // to avoid redundant iterative trigonometry calculations inside the channel loop.
        // Impact: Reduces CPU overhead per time step significantly.
        let currentLLH = MathUtils.xyz2llh(currentXYZ)
        let currentTMat = MathUtils.ltcmat(currentLLH)

        var gains = [Int](repeating: 0, count: Constants.MAX_CHAN)
        
        for i in 0..<Constants.MAX_CHAN {
            if channels[i].prn > 0 {
                let svIdx = channels[i].prn - 1
                let satData = Positioning.calculateSatPos(eph: eph[ieph][svIdx], g: receiverTime)
                let rho = Channel.estimateRange(ionoutc: ionUTC, g: receiverTime, xyz: currentXYZ, satPos: satData.pos, satVel: satData.vel, clk: satData.clk, llh: currentLLH, tmat: currentTMat)
                
                GPSSignal.updateCodePhase(chan: &channels[i], rho1: rho, dt: Constants.TIME_STEP, delt: samplePeriod)
                
                let carrierPhaseScale = 512.0 * 65536.0
                channels[i].carrPhaseStep = Int(round(carrierPhaseScale * channels[i].fCarr * samplePeriod))
                
                let elevOffset = 90.0
                let elevBinSize = 5.0
                var elevBinIdx = Int((elevOffset - rho.azel.el * Constants.R2D) / elevBinSize)
                if elevBinIdx < 0 { elevBinIdx = 0 }
                else if elevBinIdx >= 37 { elevBinIdx = 36 }
                
                let signalGainBase = 20200000.0
                let iqGainScale = 128.0
                gains[i] = Int(signalGainBase / rho.d * antPat[elevBinIdx] * iqGainScale)
            }
        }
        
        let active = (0..<Constants.MAX_CHAN).filter { channels[$0].prn > 0 }
        let iqBuffSize = Int(config.sampFreq * Constants.TIME_STEP)
        var iqBuff = [Int16](repeating: 0, count: 2 * iqBuffSize)
        
        GPSSignal.generateSamples(iqBuff: &iqBuff, iqBuffSize: iqBuffSize, channels: &channels, gains: gains, active: active, iAcc: &iAccBuff, qAcc: &qAccBuff)
        
        // Every 30 seconds, update navigation message and visibility
        if stepIdx > 0 && Int(receiverTime.sec * 10.0 + 0.5) % 3000 == 0 {
            for i in 0..<Constants.MAX_CHAN {
                if channels[i].prn > 0 {
                    channels[i].generateNavMsg(g: receiverTime, initFlag: false)
                }
            }
            allocateChannels()
        }
        
        receiverTime = receiverTime.adding(seconds: Constants.TIME_STEP)
        return iqBuff
    }
}
