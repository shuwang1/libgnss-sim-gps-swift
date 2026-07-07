# gps-sig-gen-swift: Modular GNSS Signal Generator

[![CI](https://github.com/shuwang1/gps-signal-generator/actions/workflows/ci.yml/badge.svg)](https://github.com/shuwang1/gps-signal-generator/actions/workflows/ci.yml)[![codecov](https://codecov.io/github/shuwang1/libgnss-gps-sim/graph/badge.svg?token=KWS4NXYUT6)](https://codecov.io/github/shuwang1/libgnss-gps-sim)[![Doc Pages-build-deployment](https://github.com/shuwang1/gps-signal-generator/actions/workflows/pages/pages-build-deployment/badge.svg)](https://github.com/shuwang1/gps-signal-generator/actions/workflows/pages/pages-build-deployment)

A high-performance Swift implementation of a GNSS L1 C/A baseband signal generator, ported from the original **Oriental AI** internel C project. This tool generates raw I/Q samples for SDR (Software Defined Radio) hardware, supporting custom trajectories and RINEX navigation data.

## Features

- **Multi-constellation support**: Foundation for GPS.
- **Accurate Physics**: WGS84 Earth model, Klobuchar ionospheric delay, and relativistic clock corrections.
- **Trajectory Processing**: Support for ECEF/LLH CSV files and NMEA GGA streams.
- **High Performance**: Optimized fixed-point arithmetic for signal synthesis.
- **Swift 6 Concurrency**: Built with strict concurrency safety for modern environments.
- **Rich Logging**: Color-coded, context-aware logging with simulation time tracking.

## Installation

Ensure you have Swift 6.0 or later installed on your macOS (10.15.4+) or Linux system.

```bash
git clone <repository-url>
cd gps-signal-generator
swift build -c release
```

## Usage

Generate a 1-minute signal file at a static location:

```bash
./.build/release/gnss-sig-gen-swift -e brdc0010.22n -d 60 -o signal.bin
```

### Command Line Options

| Option | Short | Description | Default |
| :--- | :--- | :--- | :--- |
| `--ephemeris` | `-e` | Path to RINEX navigation file | (Required) |
| `--user-motion` | `-u` | ECEF motion CSV (t,x,y,z) | Static Mode |
| `--llh-motion` | | LLH motion CSV (t,lat,lon,h) | |
| `--nmea-gga` | | NMEA GGA stream file | |
| `--output` | `-o` | Output signal file path | `gpssim.bin` |
| `--samp-freq` | `-s` | Sampling frequency (Hz) | `2600000.0` |
| `--elv-mask` | | Elevation mask (degrees) | `5.0` |
| `--duration` | `-d` | Simulation duration (seconds) | `300` |
| `--bits` | `-b` | I/Q format (1, 8, or 16) | `16` |
| `--verbose` | `-v` | Enable debug logging | `false` |

## Documentation

The codebase is documented using Swift-native DocC style. You can generate a documentation catalog using:

```bash
swift package generate-documentation
```

## License

**Proprietary and Confidential.** Refer to [LICENSE.md](LICENSE.md) for full licensing details. All rights reserved by Shu Wang.
