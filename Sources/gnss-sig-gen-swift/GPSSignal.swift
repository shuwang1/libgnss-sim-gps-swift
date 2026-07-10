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

/// Core signal processing for GNSS baseband generation.
struct GPSSignal {
    /// Updates the code and carrier phases for a channel based on receiver motion.
    /// - Parameters:
    ///   - chan: The channel link to update.
    ///   - rho1: The new estimated range parameters.
    ///   - dt: Simulation time step.
    ///   - delt: Sample period.
    static func updateCodePhase(chan: inout Link, rho1: Range, dt: Double, delt: Double) {
        var fCenter = Constants.FREQ_GPS_L1
        var fCodeBase = Constants.CODE_FREQ
        
        if chan.sys == .glo {
            fCenter = Constants.FREQ_GLO_L1 + Double(chan.gloFreqK) * Constants.FREQ_GLO_L1_STEP
            fCodeBase = 0.511e6 // GLONASS G1 chip rate
        } else if chan.sys == .bds {
            fCenter = Constants.FREQ_BDS_B1I
            fCodeBase = 2.046e6 // BeiDou B1I chip rate
        }
        
        let lambda = Constants.SPEED_OF_LIGHT / fCenter
        chan.fCarr = (chan.rho0.range - rho1.range) / (dt * lambda)
        chan.fCode = fCodeBase + chan.fCarr * (fCodeBase / fCenter)
        
        let ms = ((chan.rho0.g - chan.g0) + 6.0 - chan.rho0.range / Constants.SPEED_OF_LIGHT) * 1000.0
        let ims = Int(floor(ms))
        chan.codePhase = (ms - Double(ims)) * Double(chan.codeLength)
        
        var msVal = ims
        chan.iword = msVal / 600
        msVal -= chan.iword * 600
        chan.ibit = msVal / 20
        msVal -= chan.ibit * 20
        chan.icode = msVal
        
        chan.codePhaseFixed = UInt64(chan.codePhase * 4294967296.0)
        chan.codePhaseStep = UInt64(chan.fCode * delt * 4294967296.0)
        
        let chipIdx = Int(chan.codePhase)
        chan.codeCA = Int(((chan.ca[chipIdx >> 5] >> (chipIdx & 0x1F)) & 1) << 1) - 1
        
        chan.dataBit = Int((chan.dwrd[chan.iword] >> (29 - chan.ibit)) & 0x1) * 2 - 1
        chan.rho0 = rho1
    }
    
    /// Generates interleaved I/Q samples for the active channels.
    /// - Parameters:
    ///   - iqBuff: The output buffer for I/Q samples.
    ///   - iqBuffSize: Number of complex samples to generate.
    ///   - channels: Array of all simulation channels.
    ///   - gains: Signal gain for each channel.
    ///   - active: Indices of active channels to synthesize.
    ///   - iAcc: Pre-allocated buffer for I channel accumulation.
    ///   - qAcc: Pre-allocated buffer for Q channel accumulation.
    static func generateSamples(iqBuff: inout [Int16], iqBuffSize: Int, channels: inout [Link], gains: [Int], active: [Int], iAcc: inout [Int], qAcc: inout [Int]) {
        // Zero out the accumulation buffers for the current step
        iAcc.withUnsafeMutableBufferPointer { ptr in ptr.update(repeating: 0) }
        qAcc.withUnsafeMutableBufferPointer { ptr in ptr.update(repeating: 0) }
        
        // Optimization: Bypassing Swift array bounds checking using UnsafeBufferPointers.
        // The inner loops here run millions of times per second (digital signal processing).
        // Using direct memory access avoids significant CPU overhead and speeds up sample generation.
        LUT.iq_lut.withUnsafeBufferPointer { iqLutPtr in
            iAcc.withUnsafeMutableBufferPointer { iAccPtr in
                qAcc.withUnsafeMutableBufferPointer { qAccPtr in
                    for ai in active {
                        var c = channels[ai]
                        var g = gains[ai] * c.dataBit
                        var codePhase = c.codePhaseFixed
                        let codeStep = c.codePhaseStep
                        var carrPhase = c.carrPhase
                        let carrStep = UInt32(bitPattern: Int32(truncatingIfNeeded: c.carrPhaseStep))

                        var isamp = 0
                        while isamp < iqBuffSize {
                            var chip = UInt32(codePhase >> 32)
                            if chip >= UInt32(c.codeLength) {
                                codePhase -= UInt64(c.codeLength) << 32
                                chip -= UInt32(c.codeLength)
                                c.icode += 1
                                if c.icode >= 20 {
                                    c.icode = 0
                                    c.ibit += 1
                                    if c.ibit >= 30 {
                                        c.ibit = 0
                                        c.iword += 1
                                    }
                                    c.dataBit = Int((c.dwrd[c.iword] >> (29 - c.ibit)) & 0x1) * 2 - 1
                                    g = gains[ai] * c.dataBit
                                }
                            }

                            let remainingFixed = (UInt64(chip + 1) << 32) - codePhase
                            let nSamplesInChip = Int((remainingFixed + codeStep - 1) / codeStep)
                            var nToDo = iqBuffSize - isamp
                            if nToDo > nSamplesInChip { nToDo = nSamplesInChip }

                            let p_val = Int(((c.ca[Int(chip) >> 5] >> (Int(chip) & 0x1F)) & 1) << 1) - 1
                            var p = p_val
                            if c.mod == .boc11 {
                                let subPhase = UInt32(codePhase >> 31)
                                if (subPhase & 1) != 0 { p = -p }
                            }
                            p *= g

                            for _ in 0..<nToDo {
                                let idx = Int((carrPhase >> 15) & 0x3fe)
                                iAccPtr[isamp] += p * iqLutPtr[idx]
                                qAccPtr[isamp] += p * iqLutPtr[idx + 1]
                                carrPhase = carrPhase &+ carrStep
                                codePhase += codeStep
                                isamp += 1
                            }
                        }
                        c.codePhaseFixed = codePhase
                        c.carrPhase = carrPhase
                        let currentChip = Int(codePhase >> 32)
                        c.codeCA = Int(((c.ca[currentChip >> 5] >> (currentChip & 0x1F)) & 1) << 1) - 1
                        channels[ai] = c
                    }
                }
            }
        }
        
        iqBuff.withUnsafeMutableBufferPointer { iqBuffPtr in
            for isamp in 0..<iqBuffSize {
                iqBuffPtr[isamp << 1] = Int16(truncatingIfNeeded: (iAcc[isamp] + 64) >> 7)
                iqBuffPtr[(isamp << 1) + 1] = Int16(truncatingIfNeeded: (qAcc[isamp] + 64) >> 7)
            }
        }
    }
}
