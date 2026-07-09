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

/// Represents the calculated range between a receiver and a satellite.
struct Range {
    /// GPS time of the range measurement.
    var g: GPSTime = GPSTime(week: 0, sec: 0)
    /// Geometric range plus clock bias and atmospheric delays (meters).
    var range: Double = 0
    /// Rate of change of the range (Doppler shift, m/s).
    var rate: Double = 0
    /// Geometric distance between satellite and receiver (meters).
    var d: Double = 0
    /// Azimuth and Elevation of the satellite relative to the receiver (radians).
    var azel: (az: Double, el: Double) = (0, 0)
    /// Calculated ionospheric delay correction (meters).
    var ionoCorrection: Double = 0
}

/// Simulation of signal propagation channels.
struct Channel {
    
    /// Estimates the ionospheric delay using the Klobuchar model.
    /// - Parameters:
    ///   - ionoutc: Ionospheric and UTC parameters from RINEX.
    ///   - g: Current GPS time.
    ///   - llh: Receiver LLH position.
    ///   - azel: Satellite Azimuth and Elevation.
    /// - Returns: Ionospheric correction in meters.
    static func estimateIonosphericCorrection(ionoutc: IonUTC, g: GPSTime, llh: Vector3, azel: (az: Double, el: Double)) -> Double {
        if !ionoutc.enable { return 0.0 }
        let E = azel.el / Constants.PI
        let phi_u = llh[0] / Constants.PI
        let lam_u = llh[1] / Constants.PI
        let fTerm = 0.53 - E
        let F = 1.0 + 16.0 * (fTerm * fTerm * fTerm)
        
        if !ionoutc.vflg { return F * 5.0e-9 * Constants.SPEED_OF_LIGHT }
        
        let psi = 0.0137 / (E + 0.11) - 0.022
        var phi_i = phi_u + psi * cos(azel.az)
        if phi_i > 0.416 { phi_i = 0.416 } else if phi_i < -0.416 { phi_i = -0.416 }
        
        let lam_i = lam_u + psi * sin(azel.az) / cos(phi_i * Constants.PI)
        let phi_m = phi_i + 0.064 * cos((lam_i - 1.617) * Constants.PI)
        
        // Horner's method for polynomial evaluation
        var AMP = ionoutc.alpha0 + phi_m * (ionoutc.alpha1 + phi_m * (ionoutc.alpha2 + phi_m * ionoutc.alpha3))
        if AMP < 0.0 { AMP = 0.0 }
        
        // Horner's method for polynomial evaluation
        var PER = ionoutc.beta0 + phi_m * (ionoutc.beta1 + phi_m * (ionoutc.beta2 + phi_m * ionoutc.beta3))
        if PER < 72000.0 { PER = 72000.0 }
        
        var t = Constants.SECONDS_IN_DAY / 2.0 * lam_i + g.sec
        while t >= Constants.SECONDS_IN_DAY { t -= Constants.SECONDS_IN_DAY }
        while t < 0 { t += Constants.SECONDS_IN_DAY }
        
        let X = 2.0 * Constants.PI * (t - 50400.0) / PER
        if abs(X) < 1.57 {
            let X2 = X * X
            return F * (5.0e-9 + AMP * (1.0 - X2 / 2.0 + X2 * X2 / 24.0)) * Constants.SPEED_OF_LIGHT
        }
        return F * 5.0e-9 * Constants.SPEED_OF_LIGHT
    }
    
    /// Estimates the pseudorange and related parameters for a satellite-receiver pair.
    /// - Parameters:
    ///   - ionoutc: Ionospheric parameters.
    ///   - g: Receiver GPS time.
    ///   - xyz: Receiver ECEF position.
    ///   - satPos: Satellite ECEF position.
    ///   - satVel: Satellite ECEF velocity.
    ///   - clk: Satellite clock bias and drift.
    /// - Returns: A `Range` instance with estimated signal parameters.
    static func estimateRange(ionoutc: IonUTC, g: GPSTime, xyz: Vector3, satPos: Vector3, satVel: Vector3, clk: (bias: Double, drift: Double)) -> Range {
        var rho = Range()
        var pos = satPos
        
        let los0 = pos - xyz
        let tau = length(los0) / Constants.SPEED_OF_LIGHT
        
        // Account for Earth's rotation during signal flight time
        pos -= satVel * tau
        let xrot = pos.x + pos.y * Constants.OMEGA_EARTH * tau
        let yrot = pos.y - pos.x * Constants.OMEGA_EARTH * tau
        pos.x = xrot
        pos.y = yrot
        
        let los = pos - xyz
        let range = length(los)
        rho.d = range
        rho.range = range - Constants.SPEED_OF_LIGHT * clk.bias
        rho.rate = dot(satVel, los) / range
        rho.g = g
        
        let llh = MathUtils.xyz2llh(xyz)
        let tmat = MathUtils.ltcmat(llh)
        let neu = MathUtils.ecef2neu(los, t: tmat)
        rho.azel = MathUtils.neu2azel(neu)
        
        rho.ionoCorrection = estimateIonosphericCorrection(ionoutc: ionoutc, g: g, llh: llh, azel: rho.azel)
        rho.range += rho.ionoCorrection
        
        return rho
    }
}
