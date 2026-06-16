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

import Testing
import Foundation
@testable import gnss_sig_gen_swift

/// Unit tests for signal propagation channel modeling.
struct ChannelTests {

    /// Verifies range estimation including corrections for signal flight time and Earth rotation.
    @Test func testRangeEstimation() async throws {
        let ionoutc = IonUTC() 
        let g = GPSTime(week: 2000, sec: 0.0)
        let xyz = Vector3(Constants.WGS84_RADIUS, 0, 0) // Receiver at equator
        
        let satPos = Vector3(Constants.WGS84_RADIUS + 20000000.0, 0, 0) // Sat directly above
        let satVel = Vector3(0, 3000.0, 0)
        let clk = (bias: 0.0, drift: 0.0)
        
        let llh = MathUtils.xyz2llh(xyz)
        let tmat = MathUtils.ltcmat(llh)
        let rho = Channel.estimateRange(ionoutc: ionoutc, g: g, xyz: xyz, satPos: satPos, satVel: satVel, clk: clk, llh: llh, tmat: tmat)
        
        #expect(abs(rho.d - 20000000.0) < 100.0) 
        #expect(abs(rho.azel.el - 90.0 * Constants.D2R) < 1.0 * Constants.D2R)
    }
    
    /// Verifies ionospheric delay estimation using the Klobuchar model.
    @Test func testIonoCorrection() async throws {
        var ionoutc = IonUTC()
        ionoutc.vflg = false // Triggers default 5ns delay
        ionoutc.enable = true
        
        let g = GPSTime(week: 2000, sec: 0.0)
        let llh = Vector3(0, 0, 0)
        let azel = (az: 0.0, el: 90.0 * Constants.D2R)
        
        let corr = Channel.estimateIonosphericCorrection(ionoutc: ionoutc, g: g, llh: llh, azel: azel)
        // F = 1.0 for el = 90
        #expect(abs(corr - 5.0e-9 * Constants.SPEED_OF_LIGHT) < 1e-3)
    }
}
