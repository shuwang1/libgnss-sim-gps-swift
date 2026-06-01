# Installation and Usage Guide

This guide provides instructions on how to install, build, run, and test the `gnss-sig-gen-swift` package.

## Prerequisites

- **Operating System**: macOS 10.15.4 or newer (Recommended). Linux is also supported but macOS is the primary target for CI.
- **Swift Toolchain**: Swift 6.0 or later.
  - On Linux, you can install it via [swiftly](https://github.com/swiftly-step/swiftly) or from [Swift.org](https://swift.org/download/).
  - On macOS, install Xcode 16.0 or later.

## Installation

1.  **Clone the repository**:
    ```bash
    git clone <repository-url>
    cd gnss-sig-gen-swift
    ```

2.  **Build the project**:
    - For development/debugging:
      ```bash
      swift build
      ```
    - For a high-performance production build (recommended for signal generation):
      ```bash
      swift build -c release
      ```

The compiled binary will be located at `.build/release/gnss-sig-gen-swift`.

## Execution

### Basic Usage

To generate a 10-second GPS signal file using a RINEX navigation file:

```bash
swift run gnss-sig-gen-swift -e path/to/ephemeris.22n -d 10 -o output.bin
```

### Advanced Examples

1.  **Simulate with User Trajectory (ECEF CSV)**:
    ```bash
    swift run gnss-sig-gen-swift -e brdc0010.22n -u trajectory.csv -o signal_ecef.bin
    ```
    *CSV Format: `time, x, y, z` (meters)*

2.  **Simulate with LLH Trajectory**:
    ```bash
    swift run gnss-sig-gen-swift -e brdc0010.22n --llh-motion path.csv -o signal_llh.bin
    ```
    *CSV Format: `time, lat, lon, height` (degrees, degrees, meters)*

3.  **High Sampling Rate (e.g., for BladeRF/HackRF)**:
    ```bash
    swift run gnss-sig-gen-swift -e brdc0010.22n -s 5000000 -d 30 -o fast_signal.bin
    ```

### I/Q Output Format
The output file contains interleaved **16-bit signed integers** (SC16 format) by default:
`[I0, Q0, I1, Q1, ...]`

## Unit Testing

The project includes a comprehensive suite of unit tests verifying mathematical correctness and data parsing.

### Run All Tests
```bash
swift test
```

### Run Specific Test Suite
```bash
swift test --filter MathTests
```

### Generate Test Coverage Report
```bash
swift test --enable-code-coverage
```

## Documentation

Generate and view the interactive API documentation (DocC):

```bash
swift package generate-documentation --target gnss-sig-gen-swift --open
```
