# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0] - 2026-05-31

### Added
- **Initial Swift Port**: Completed the transition from C to a modular Swift package.
- **Explicit Licensing**: Added detailed license and liability clauses to every source file and `LICENSE.md`, ensuring maximum legal clarity and protection.
- **Core GNSS Logic**:
  - RINEX 2.x and 3.x navigation message parsing (`GPSEphemeris.swift`).
  - GPS L1 C/A Gold code generation with performance-optimized bit mapping (`GPSCode.swift`).
  - High-precision satellite positioning and velocity calculations (`Positioning.swift`).
  - Klobuchar ionospheric delay modeling and range estimation (`Channel.swift`).
  - WGS84 coordinate transformations (ECEF, LLH, NEU) (`MathUtils.swift`).
- **Rich Logging System**:
  - Color-coded terminal output with wall-clock and simulation timestamps.
  - Granular log levels (`debug`, `info`, `warn`, `error`) controlled via CLI flags.
  - Hexadecimal data dump utility for low-level debugging.
- **Modern CLI Interface**:
  - Built with `swift-argument-parser` for a robust and user-friendly experience.
  - Supports multiple trajectory formats (ECEF CSV, LLH CSV, NMEA GGA).
- **Comprehensive Unit Testing**:
  - 16 targeted tests across 8 suites using the new `Swift Testing` framework.
  - Mathematical verification of orbital and coordinate transformations.
- **CI/CD Workflows**:
  - GitHub Actions for automated building and testing on Linux.
  - Automated API documentation deployment via GitHub Pages.

### Changed
- **Performance Optimization**: Ported signal synthesis logic to use fixed-point arithmetic, mirroring the performance characteristics of the original C implementation.
- **Documentation**: Transitioned all internal documentation to Swift-native DocC format for automated catalog generation.

### Fixed
- **Integration Readiness**: Resolved several critical bugs identified during successful integration testing with the Erlang GNSS receiver:
  - **Signal Scaling**: Corrected 8-bit signal scaling (divide by 16) to match reference C implementation and prevent clipping.
  - **Doppler Handling**: Fixed a trap when casting negative Doppler values to `UInt32` by using bit-pattern initialization.
  - **Static Trajectories**: Improved `Simulator` logic to handle single-point trajectory files correctly over a specified duration.
  - **Parsing Robustness**: Implemented whitespace trimming for CSV trajectory fields.
- **Documentation System**: Fixed errors in documentation generation by adding `swift-docc-plugin`, creating a DocC catalog landing page, and completing missing API parameter documentation.
- **Trajectory Parsing**: Fixed a bug where leading/trailing whitespace in CSV files caused parsing failures; implemented automatic field trimming.
- **Linux Compatibility**: Implemented a custom `Vector3` structure to eliminate dependency on the Apple-only `simd` framework, ensuring full portability.

## [Pre-release]

- Initial architecture design and feasibility study for the C-to-Swift port.
