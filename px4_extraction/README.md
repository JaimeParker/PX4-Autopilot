# PX4 Attitude Controller Extraction

Standalone C++ implementation of PX4's quaternion-based attitude controller for use in reinforcement learning environments.

## Overview

This project extracts the core attitude control algorithm from PX4-Autopilot v1.16.0, removing all PX4 framework dependencies (uORB, WorkItem, etc.) and providing a clean API for autonomous flight control.

**Controller Type:** Quaternion-based proportional attitude controller
**Algorithm:** Based on "Nonlinear Quadrocopter Attitude Control" (ETH Zurich, 2013)

## Features

- Pure C++17 implementation
- No PX4 framework dependencies
- Autonomous mode only (no manual control code)
- Default PX4 gains with runtime configuration
- Includes PX4 matrix library for quaternion math
- Simple test suite

## Directory Structure

```
px4_extraction/
├── CMakeLists.txt                    # Root build configuration
├── README.md                         # This file
├── attitude_control/                 # Core controller
│   ├── AttitudeControl.hpp           # Core algorithm (from PX4)
│   ├── AttitudeControl.cpp           # Core implementation (from PX4)
│   ├── AttitudeControlTypes.hpp      # Data structures
│   ├── AttitudeControllerWrapper.hpp # Clean API
│   ├── AttitudeControllerWrapper.cpp # Wrapper implementation
│   └── CMakeLists.txt
├── lib/                              # Dependencies
│   ├── matrix/                       # PX4 matrix library (quaternion, vector math)
│   └── mathlib/                      # Math utilities (Limits, Functions)
└── test/                             # Tests
    ├── attitude_control_simple_test.cpp
    └── CMakeLists.txt
```

## Building

### Prerequisites

- CMake 3.10 or higher
- C++17 compatible compiler (GCC 7+, Clang 5+)
- No external dependencies required (matrix library included)

### Build Instructions

```bash
cd px4_extraction
mkdir build && cd build
cmake ..
make
```

### Run Tests

```bash
./test/attitude_control_simple_test
```

## Usage Example

```cpp
#include "AttitudeControllerWrapper.hpp"

using namespace px4_extraction;

// Create controller with default PX4 gains
AttitudeControllerWrapper controller;

// Optional: Configure custom gains
AttitudeControlConfig config;
config.roll_p = 6.5f;
config.pitch_p = 6.5f;
config.yaw_p = 2.8f;
config.yaw_weight = 0.4f;
config.max_roll_rate = 3.84f;   // rad/s
config.max_pitch_rate = 3.84f;  // rad/s
config.max_yaw_rate = 3.49f;    // rad/s
controller.setGains(config);

// Prepare inputs
AttitudeState state;
state.q = matrix::Quatf(1.0f, 0.0f, 0.0f, 0.0f);  // Current attitude (identity)
state.timestamp = 0;

AttitudeSetpoint setpoint;
setpoint.q_d = matrix::Quatf(0.9914f, 0.1305f, 0.0f, 0.0f);  // 15° roll
setpoint.yaw_sp_move_rate = 0.0f;
setpoint.thrust_body = matrix::Vector3f(0.0f, 0.0f, -0.5f);  // 50% thrust

// Run controller
RateSetpoint output = controller.update(state, setpoint);

// Use outputs
std::cout << "Roll rate: " << output.rates(0) << " rad/s" << std::endl;
std::cout << "Pitch rate: " << output.rates(1) << " rad/s" << std::endl;
std::cout << "Yaw rate: " << output.rates(2) << " rad/s" << std::endl;
```

## API Reference

### Data Structures

#### `AttitudeControlConfig`
Controller configuration (gains and limits)
- `float roll_p` - Roll proportional gain (default: 6.5)
- `float pitch_p` - Pitch proportional gain (default: 6.5)
- `float yaw_p` - Yaw proportional gain (default: 2.8)
- `float yaw_weight` - Yaw deprioritization [0-1] (default: 0.4)
- `float max_roll_rate` - Max roll rate rad/s (default: 3.84)
- `float max_pitch_rate` - Max pitch rate rad/s (default: 3.84)
- `float max_yaw_rate` - Max yaw rate rad/s (default: 3.49)

#### `AttitudeState`
Current vehicle attitude
- `uint64_t timestamp` - Time in microseconds
- `matrix::Quatf q` - Current attitude quaternion (Hamilton: w,x,y,z)

#### `AttitudeSetpoint`
Desired attitude from position controller
- `uint64_t timestamp` - Time in microseconds
- `matrix::Quatf q_d` - Desired attitude quaternion
- `float yaw_sp_move_rate` - Feedforward yaw rate [rad/s]
- `matrix::Vector3f thrust_body` - Thrust vector [-1, 1]

#### `RateSetpoint`
Output to rate controller
- `uint64_t timestamp` - Time in microseconds
- `matrix::Vector3f rates` - Body angular rates [rad/s]
- `matrix::Vector3f thrust_body` - Thrust vector (passthrough)

### Main Class

#### `AttitudeControllerWrapper`

**Constructor**
```cpp
AttitudeControllerWrapper()
```
Initializes with default PX4 gains

**Methods**
```cpp
void setGains(const AttitudeControlConfig &config)
```
Configure controller gains and rate limits

```cpp
AttitudeControlConfig getGains() const
```
Get current configuration

```cpp
RateSetpoint update(const AttitudeState &state, const AttitudeSetpoint &setpoint)
```
Main control loop - converts attitude setpoint to rate setpoint

```cpp
void reset()
```
Reset controller (currently no-op, placeholder for future)

## Control Flow

```
Position Controller (upstream)
    ↓
AttitudeSetpoint (quaternion + thrust + yaw_rate)
    ↓
AttitudeControllerWrapper::update()
    ↓
AttitudeControl::update() [Core PX4 algorithm]
    ↓
RateSetpoint (body rates + thrust)
    ↓
Rate Controller (downstream)
```

## Algorithm Details

The controller implements a proportional controller on quaternion error:

1. **Yaw Deprioritization**: Reduces desired attitude by yaw weight to prioritize roll/pitch
2. **Quaternion Error**: Computes `qe = q^{-1} * q_d`
3. **Attitude Error Vector**: `eq = 2 * qe.imag()` (scaled rotation axis)
4. **P Control**: `rate_sp = P_gain ⊗ eq`
5. **Yaw Feedforward**: Adds `yaw_sp_move_rate` transformed to body frame
6. **Rate Limiting**: Constrains output to max rate parameters

## Default Parameters

Based on PX4 v1.16.0 defaults:

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| MC_ROLL_P | 6.5 | rad/s per rad | Roll P gain |
| MC_PITCH_P | 6.5 | rad/s per rad | Pitch P gain |
| MC_YAW_P | 2.8 | rad/s per rad | Yaw P gain |
| MC_YAW_WEIGHT | 0.4 | - | Yaw weight [0-1] |
| MC_ROLLRATE_MAX | 220 | °/s | Max roll rate (3.84 rad/s) |
| MC_PITCHRATE_MAX | 220 | °/s | Max pitch rate (3.84 rad/s) |
| MC_YAWRATE_MAX | 200 | °/s | Max yaw rate (3.49 rad/s) |

## License

BSD 3-Clause (same as PX4)

## References

- **PX4-Autopilot**: https://github.com/PX4/PX4-Autopilot
- **Algorithm Paper**: [Nonlinear Quadrocopter Attitude Control (2013)](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf)
- **PX4 Documentation**: https://docs.px4.io/

## Notes

- This extraction is for **autonomous mode only** - no manual control, VTOL, or EKF reset handling
- The matrix library is included but users can replace it with Eigen3 if preferred
- Controller is stateless (no integral terms at this level)
- Next step: Extract rate controller and control allocator for complete control chain
