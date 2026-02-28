# PX4 Inner-Loop Controller Extraction

Standalone C++ implementation of PX4's attitude controller, body-rate controller, control allocator, and actuator output mapping for use in reinforcement learning environments.

## Overview

This project extracts the core inner-loop control algorithms from PX4-Autopilot v1.16.0, removing all PX4 framework dependencies (uORB, WorkItem, parameters, etc.) and providing a clean, data-driven API for autonomous quadrotor control.

**Attitude Controller:** Quaternion-based proportional controller — "Nonlinear Quadrocopter Attitude Control" (ETH Zurich, 2013)
**Rate Controller:** 3-axis PID body-rate controller with integrator anti-windup and yaw torque LPF
**Control Allocator:** Sequential desaturation mixer with configurable airmode and Iris quadrotor geometry
**Actuator Output:** Thrust curve linearization (THR_MDL_FAC inverse) with PWM and DShot output mapping

## Features

- Pure C++17 implementation
- No PX4 framework dependencies (no uORB, no param system, no work queues)
- Autonomous mode only (no manual/acro control code)
- Default PX4 gains with full runtime configuration
- Includes PX4 matrix library for quaternion math
- AlphaFilter (first-order LPF) for yaw torque smoothing
- Integrator anti-windup with control-allocator saturation feedback
- Sequential desaturation with 3 airmode variants (disabled, RP, RPY)
- Configurable rotor geometry (Iris preset + custom)
- Thrust model factor inverse curve for motor response linearization
- PWM (1000-2000 µs) and DShot (1-1999) output modes
- Comprehensive GTest suite (69 tests)

## Directory Structure

```
px4_extraction/
├── CMakeLists.txt                        # Root build configuration
├── README.md                             # This file
├── attitude_control/                     # Attitude controller (Phase 1)
│   ├── AttitudeControl.hpp               # Core algorithm (from PX4)
│   ├── AttitudeControl.cpp               # Core implementation (from PX4)
│   ├── AttitudeControlTypes.hpp          # Data structures
│   ├── AttitudeControllerWrapper.hpp     # Clean API wrapper
│   ├── AttitudeControllerWrapper.cpp     # Wrapper implementation
│   └── CMakeLists.txt
├── rate_control/                         # Rate controller (Phase 2)
│   ├── RateControl.hpp                   # Core PID algorithm (from PX4)
│   ├── RateControl.cpp                   # Core implementation (from PX4)
│   ├── RateControlTypes.hpp              # Params, I/O structs, status
│   ├── RateControllerWrapper.hpp         # Clean API with K-scaling + yaw LPF
│   ├── RateControllerWrapper.cpp         # Wrapper implementation
│   └── CMakeLists.txt
├── control_allocator/                    # Control allocator (Phase 3)
│   ├── ActuatorEffectiveness.hpp         # Base interface (NUM_ACTUATORS, axes, config)
│   ├── ActuatorEffectiveness.cpp         # Configuration helpers
│   ├── ControlAllocation.hpp             # Base allocator (bounds, clipping, slew rate)
│   ├── ControlAllocation.cpp             # Base implementation
│   ├── ControlAllocationPseudoInverse.hpp # Pseudo-inverse allocator
│   ├── ControlAllocationPseudoInverse.cpp # Mix matrix + RPY normalization
│   ├── ControlAllocationSequentialDesaturation.hpp # Desaturation + airmode
│   ├── ControlAllocationSequentialDesaturation.cpp # Full desaturation algorithm
│   ├── RotorGeometry.hpp                 # Geometry structs + effectiveness matrix + presets
│   ├── ControlAllocatorTypes.hpp         # Params, I/O structs
│   ├── ControlAllocatorWrapper.hpp       # Clean API wrapper
│   ├── ControlAllocatorWrapper.cpp       # Wrapper implementation
│   └── CMakeLists.txt
├── actuator_output/                      # Actuator output mapping (Phase 4)
│   ├── ActuatorOutputTypes.hpp           # OutputMode enum, params, I/O structs
│   ├── ThrustCurve.hpp                   # THR_MDL_FAC inverse (header-only)
│   ├── ActuatorOutputWrapper.hpp         # Clean API wrapper
│   ├── ActuatorOutputWrapper.cpp         # PWM + DShot mapping implementation
│   └── CMakeLists.txt
├── lib/                                  # Dependencies
│   ├── matrix/                           # PX4 matrix library (quaternion, vector math)
│   ├── mathlib/                          # Math utilities (Limits, Functions, AlphaFilter)
│   └── px4_platform_common/              # Minimal shim (PX4_ISFINITE, M_TWOPI_F)
└── test/                                 # Tests
    ├── attitude_control_simple_test.cpp  # Simple attitude controller demo
    ├── AttitudeControlTest.cpp           # GTest: attitude control (3 tests)
    ├── RateControlTest.cpp               # GTest: rate control + pipeline (16 tests)
    ├── ControlAllocatorTest.cpp          # GTest: allocator + full pipeline (22 tests)
    ├── ActuatorOutputTest.cpp            # GTest: thrust curve, PWM, DShot, pipeline (28 tests)
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
# Attitude controller tests (3 GTests + convergence suite)
./test/attitude_control_gtest

# Rate controller + pipeline tests (16 GTests)
./test/rate_control_gtest

# Control allocator + full pipeline tests (22 GTests)
./test/control_allocator_gtest

# Actuator output + thrust curve + full 4-phase pipeline tests (28 GTests)
./test/actuator_output_gtest

# Simple attitude controller demo
./test/attitude_control_simple_test
```

## Usage Example

### Attitude Controller Only

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

AttitudeSetpoint setpoint;
setpoint.q_d = matrix::Quatf(0.9914f, 0.1305f, 0.0f, 0.0f);  // 15° roll
setpoint.yaw_sp_move_rate = 0.0f;
setpoint.thrust_body = matrix::Vector3f(0.0f, 0.0f, -0.5f);  // 50% thrust

// Run controller
RateSetpoint output = controller.update(state, setpoint);
// output.rates = body rate setpoint [rad/s]
// output.thrust_body = thrust passthrough
```

### Full Pipeline: Attitude → Rate → Torque

```cpp
#include "AttitudeControllerWrapper.hpp"
#include "RateControllerWrapper.hpp"

using namespace px4_extraction;

// Create both controllers with default PX4 gains
AttitudeControllerWrapper att_ctrl;
RateControllerWrapper rate_ctrl;

// Optional: tune rate controller
RateControlParams rate_params;
rate_params.gain_p = {0.15f, 0.15f, 0.2f};  // P gains [roll, pitch, yaw]
rate_params.gain_i = {0.2f, 0.2f, 0.1f};    // I gains
rate_params.gain_d = {0.003f, 0.003f, 0.0f}; // D gains
rate_params.gain_k = {1.0f, 1.0f, 1.0f};    // Global K multiplier
rate_params.integrator_limit = {0.3f, 0.3f, 0.3f};
rate_params.yaw_torque_lpf_cutoff_freq = 2.0f; // Hz
rate_ctrl.setParams(rate_params);

// === Each simulation step ===

// Step 1: Attitude controller → rate setpoint
AttitudeState state;
state.q = current_quaternion;  // from physics sim

AttitudeSetpoint att_sp;
att_sp.q_d = desired_quaternion;  // from RL policy
att_sp.thrust_body = {0.f, 0.f, -thrust};

RateSetpoint rate_sp = att_ctrl.update(state, att_sp);

// Step 2: Rate controller → torque setpoint
RateControlInput rc_input;
rc_input.rates_sp = rate_sp.rates;
rc_input.rates_actual = current_gyro;       // from physics sim
rc_input.angular_accel = current_ang_accel;  // from sim or finite-diff of gyro
rc_input.thrust_body = rate_sp.thrust_body;
rc_input.dt = sim_dt;                        // time step from RL env
rc_input.landed = false;

RateControlOutput rc_output;
rate_ctrl.update(rc_input, rc_output);
// rc_output.torque_sp = normalized torque [-1,1] for [roll, pitch, yaw]
// rc_output.thrust_body = thrust passthrough → feed to control allocator
```

### Full Pipeline: Attitude → Rate → Allocator → Motors

```cpp
#include "AttitudeControllerWrapper.hpp"
#include "RateControllerWrapper.hpp"
#include "ControlAllocatorWrapper.hpp"

using namespace px4_extraction;

// Create all controllers with default PX4 gains + Iris geometry
AttitudeControllerWrapper att_ctrl;
RateControllerWrapper rate_ctrl;
ControlAllocatorWrapper allocator;

// Optional: configure allocator
ControlAllocatorParams alloc_params;
alloc_params.geometry = createQuadXIris();  // or createQuadXGeneric()
alloc_params.airmode = 1;                   // 0=disabled, 1=RP, 2=RPY
alloc_params.normalize_rpy = true;
allocator.setParams(alloc_params);

// === Each simulation step ===

// Step 1: Attitude controller → rate setpoint
AttitudeState state;
state.q = current_quaternion;
AttitudeSetpoint att_sp;
att_sp.q_d = desired_quaternion;
att_sp.thrust_body = {0.f, 0.f, -thrust};
RateSetpoint rate_sp = att_ctrl.update(state, att_sp);

// Step 2: Rate controller → torque setpoint
RateControlInput rc_input;
rc_input.rates_sp = rate_sp.rates;
rc_input.rates_actual = current_gyro;
rc_input.angular_accel = current_ang_accel;
rc_input.thrust_body = rate_sp.thrust_body;
rc_input.dt = sim_dt;
rc_input.landed = false;
RateControlOutput rc_output;
rate_ctrl.update(rc_input, rc_output);

// Step 3: Control allocator → motor commands
ControlAllocatorInput alloc_input;
alloc_input.torque_sp = rc_output.torque_sp;
alloc_input.thrust_sp = rc_output.thrust_body;
alloc_input.dt = sim_dt;
ControlAllocatorOutput alloc_output;
allocator.update(alloc_input, alloc_output);

// Step 4: Actuator output → PWM/DShot values
#include "ActuatorOutputWrapper.hpp"
ActuatorOutputWrapper actuator;
ActuatorOutputParams act_params;
act_params.mode = OutputMode::PWM;           // or OutputMode::DShot
act_params.thrust_model_factor = 0.0f;       // 0=linear, 0.3=racer
act_params.pwm_min = 1000;
act_params.pwm_max = 2000;
act_params.pwm_disarmed = 900;
actuator.setParams(act_params);

ActuatorOutputInput act_in;
act_in.armed = true;
for (int i = 0; i < 4; ++i) act_in.motors[i] = alloc_output.motors[i];

ActuatorOutputOutput act_out;
actuator.update(act_in, act_out);
// act_out.values[0..3] = PWM [1000-2000] µs or DShot [1-1999]

// Step 5: Anti-windup feedback from allocator → rate controller
rate_ctrl.setSaturationStatus(alloc_output.saturation_positive,
                              alloc_output.saturation_negative);
```

## API Reference

### Attitude Control Data Structures

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
- `matrix::Quatf q` - Current attitude quaternion (Hamilton: w,x,y,z)

#### `AttitudeSetpoint`
Desired attitude from RL policy or position controller
- `matrix::Quatf q_d` - Desired attitude quaternion
- `float yaw_sp_move_rate` - Feedforward yaw rate [rad/s]
- `matrix::Vector3f thrust_body` - Thrust vector [-1, 1]

#### `RateSetpoint`
Output bridging attitude → rate controller
- `matrix::Vector3f rates` - Body angular rates [rad/s]
- `matrix::Vector3f thrust_body` - Thrust vector (passthrough)

### Rate Control Data Structures

#### `RateControlParams`
PID gains and filter settings (see Default Parameters table)
- `matrix::Vector3f gain_p` - Proportional gains [roll, pitch, yaw]
- `matrix::Vector3f gain_i` - Integral gains
- `matrix::Vector3f gain_d` - Derivative gains
- `matrix::Vector3f gain_ff` - Feedforward gains
- `matrix::Vector3f gain_k` - Global K multiplier (scales P, I, D together)
- `matrix::Vector3f integrator_limit` - Integral term absolute clamp
- `float yaw_torque_lpf_cutoff_freq` - Yaw torque LPF cutoff [Hz] (default: 2.0)

#### `RateControlInput`
- `matrix::Vector3f rates_sp` - Desired body rates [rad/s] (from attitude controller)
- `matrix::Vector3f rates_actual` - Measured body rates [rad/s] (from sim gyro)
- `matrix::Vector3f angular_accel` - Angular acceleration [rad/s²] (from sim or finite-diff)
- `matrix::Vector3f thrust_body` - Thrust passthrough
- `float dt` - Time step [s] (provided by RL environment)
- `bool landed` - Landed flag (freezes integrators when true)

#### `RateControlOutput`
- `matrix::Vector3f torque_sp` - Normalized torque setpoint [-1, 1]
- `matrix::Vector3f thrust_body` - Thrust passthrough

#### `RateCtrlStatus`
- `float rollspeed_integ` - Roll integrator state
- `float pitchspeed_integ` - Pitch integrator state
- `float yawspeed_integ` - Yaw integrator state

### Control Allocator Data Structures

#### `ControlAllocatorParams`
Allocator configuration (geometry, airmode, bounds)
- `Geometry geometry` - Rotor geometry (default: Iris quadrotor X)
- `int airmode` - Airmode: 0=disabled, 1=RP (default), 2=RPY
- `float actuator_min[4]` - Motor minimum values (default: 0.0)
- `float actuator_max[4]` - Motor maximum values (default: 1.0)
- `float slew_rate_limit[4]` - Motor slew rate limit (default: 0.0 = unlimited)
- `bool normalize_rpy` - Normalize RPY columns of mix matrix (default: true)

#### `ControlAllocatorInput`
- `matrix::Vector3f torque_sp` - Normalized torque setpoint [-1, 1] (from rate controller)
- `matrix::Vector3f thrust_sp` - Thrust setpoint (z-component negative = upward in NED)
- `float dt` - Time step [s] (default: 0.004)

#### `ControlAllocatorOutput`
- `float motors[4]` - Normalized motor commands [0, 1]
- `int num_motors` - Number of active motors
- `matrix::Vector3<bool> saturation_positive` - Per-axis positive saturation flags
- `matrix::Vector3<bool> saturation_negative` - Per-axis negative saturation flags
- `matrix::Vector3f unallocated_torque` - Unallocated torque (control_sp - allocated_control)

#### `Geometry` / `RotorGeometry`
- `RotorGeometry rotors[12]` - Per-rotor position, axis, thrust_coef, moment_ratio
- `int num_rotors` - Number of rotors
- Factory presets: `createQuadXIris()`, `createQuadXGeneric()`

### Actuator Output Data Structures

#### `ActuatorOutputParams`
Thrust curve and output mapping configuration
- `float thrust_model_factor` - THR_MDL_FAC [0, 1], 0 = linear (default: 0.0)
- `OutputMode mode` - Output mode: PWM or DShot (default: PWM)
- `uint16_t pwm_min` - PWM minimum µs (default: 1000)
- `uint16_t pwm_max` - PWM maximum µs (default: 2000)
- `uint16_t pwm_disarmed` - PWM disarmed µs (default: 900)
- `uint16_t dshot_min_throttle` - DShot min command (default: 1)
- `uint16_t dshot_max_throttle` - DShot max command (default: 1999)
- `uint16_t dshot_disarmed` - DShot disarmed value (default: 0)
- `float dshot_min_fraction` - DShot min fraction [0, 1] (default: 0.0)

#### `ActuatorOutputInput`
- `float motors[4]` - Normalized motor commands [0, 1] (from control allocator)
- `int num_motors` - Number of motors (default: 4)
- `bool armed` - Armed flag (false → disarmed output values)

#### `ActuatorOutputOutput`
- `uint16_t values[4]` - Output values: PWM µs or DShot command
- `int num_motors` - Number of motors

### Classes

#### `AttitudeControllerWrapper`

```cpp
AttitudeControllerWrapper()                          // Default PX4 gains
void setGains(const AttitudeControlConfig &config)   // Configure gains + rate limits
AttitudeControlConfig getGains() const               // Query current config
RateSetpoint update(const AttitudeState&, const AttitudeSetpoint&)  // Attitude → rates
void reset()                                         // Reset internal state
```

#### `RateControllerWrapper`

```cpp
RateControllerWrapper()                              // Default PX4 gains
void setParams(const RateControlParams &params)      // Configure PID + K-scaling + LPF
void update(const RateControlInput&, RateControlOutput&)  // Rates → torque
void setSaturationStatus(const matrix::Vector<bool,3> &upper,
                         const matrix::Vector<bool,3> &lower) // Anti-windup from allocator
void resetIntegral()                                 // Zero integrators + reset yaw LPF
void getStatus(RateCtrlStatus &status)               // Inspect integrator values
```

#### `ControlAllocatorWrapper`

```cpp
ControlAllocatorWrapper()                            // Default Iris geometry + RP airmode
void setParams(const ControlAllocatorParams &params) // Configure geometry, airmode, bounds
ControlAllocatorParams getParams() const             // Query current config
void update(const ControlAllocatorInput&, ControlAllocatorOutput&) // Torque+thrust → motors
const EffectivenessMatrix& getEffectivenessMatrix()  // Inspect 6×16 matrix
const ActuatorVector& getActuatorSetpoint()          // Inspect raw actuator commands
```

#### `ActuatorOutputWrapper`

```cpp
ActuatorOutputWrapper()                              // Default: linear thrust, PWM mode
void setParams(const ActuatorOutputParams &params)   // Configure thrust curve + output mode
ActuatorOutputParams getParams() const               // Query current config
void update(const ActuatorOutputInput&, ActuatorOutputOutput&) // Motors[0,1] → PWM/DShot
```

## Control Flow

```
RL Policy / Position Controller (upstream)
    ↓
AttitudeSetpoint {q_d, thrust_body, yaw_sp_move_rate}
    ↓
AttitudeControllerWrapper::update()
  └─ AttitudeControl::update()  [P on quaternion error]
    ↓
RateSetpoint {rates, thrust_body}
    ↓
RateControllerWrapper::update()
  └─ RateControl::update()  [PID + anti-windup + yaw LPF]
    ↓
RateControlOutput {torque_sp, thrust_body}
    ↓
ControlAllocatorWrapper::update()
  └─ ControlAllocationSequentialDesaturation::allocate()
     [pseudo-inverse + desaturation + airmode mixing]
    ↓
ControlAllocatorOutput {motors[4], saturation flags}
    ↓                            ↓
Motor commands [0,1]    setSaturationStatus() → rate controller anti-windup
    ↓
ActuatorOutputWrapper::update()
  └─ applyThrustCurveInverse()  [THR_MDL_FAC quadratic inverse]
  └─ mapPWM() or mapDShot()    [range mapping to hardware protocol]
    ↓
ActuatorOutputOutput {values[4]}
    ↓
PWM [1000-2000 µs] or DShot [1-1999]
```

## Algorithm Details

### Attitude Controller (Proportional on Quaternion Error)

1. **Yaw Deprioritization**: Reduces desired attitude by yaw weight to prioritize roll/pitch
2. **Quaternion Error**: $q_e = q^{-1} \otimes q_d$
3. **Attitude Error Vector**: $e_q = 2 \cdot \text{imag}(q_e)$ (scaled rotation axis)
4. **P Control**: $\omega_{sp} = K_p \odot e_q$
5. **Yaw Feedforward**: Adds `yaw_sp_move_rate` transformed to body frame
6. **Rate Limiting**: Constrains output to max rate parameters

### Rate Controller (PID with Anti-Windup)

1. **Error**: $e = \omega_{sp} - \omega_{actual}$
2. **Proportional**: $P = K_p \odot e$
3. **Integral**: $I_{n+1} = I_n + K_i \odot e \cdot i_{factor} \cdot dt$, where $i_{factor} = \max\left(0,\; 1 - \left(\frac{e}{\text{rad}(400)}\right)^2\right)$
4. **Derivative**: $D = -K_d \odot \dot{\omega}$ (uses angular acceleration, not error differentiation)
5. **Feedforward**: $FF = K_{ff} \odot \omega_{sp}$
6. **Output**: $\tau = K \odot (P + I - D + FF)$ (K-scaling converts ideal → parallel form)
7. **Anti-windup**: Integrator frozen on saturated axes (via `setSaturationStatus`) and when `landed = true`
8. **Integrator Clamping**: $|I| \leq \text{integrator\_limit}$
9. **Yaw LPF**: First-order alpha filter on yaw torque (default cutoff 2 Hz)

### Control Allocator (Sequential Desaturation)

1. **Effectiveness Matrix**: Computed from rotor geometry: $E_{6 \times N}$ where rows = [roll, pitch, yaw, thrust_x, thrust_y, thrust_z] and columns = motors
2. **Torque Computation**: For each rotor $i$ with position $p_i$, axis $a_i$, thrust coefficient $C_T$, and moment ratio $K_M$:
   - Thrust: $F_i = C_T \cdot a_i$
   - Moment: $M_i = C_T \cdot (p_i \times a_i) - C_T \cdot K_M \cdot a_i$
3. **Pseudo-Inverse**: $B = E^+ = E^T (E E^T)^{-1}$ (generalized inverse via Cholesky)
4. **RPY Normalization**: Mix matrix columns scaled so that full-scale torque input maps to full actuator range
5. **Allocation**: $u = B \cdot c_{sp}$ where $c_{sp}$ is the 6D control setpoint
6. **Sequential Desaturation** (RP airmode):
   - Mix roll+pitch torque → desaturate (shift all motors to stay in bounds)
   - Mix yaw with 15% thrust margin budget
   - Mix thrust → desaturate (preserving differential for RP authority)
7. **Desaturation Gain**: $k = \max(k_{min}, k_{max})$ where $k_{min} = \min_i(-u_i / \Delta u_i)$ and $k_{max} = \max_i((1 - u_i) / \Delta u_i)$ for actors at limits
8. **Saturation Feedback**: Unallocated torque $= c_{sp} - E \cdot u_{final}$ → fed back to rate controller integrator anti-windup

### Actuator Output (Thrust Curve + PWM/DShot Mapping)

1. **Thrust Model Forward**: $\text{thrust} = f \cdot x^2 + (1-f) \cdot x$ where $f$ = `thrust_model_factor`, $x$ = motor signal
2. **Thrust Model Inverse** (applied to normalize motor response):
   - If $f = 0$: linear passthrough ($x = \text{control}$)
   - If $f > 0$: $x = -\frac{1-f}{2f} + \sqrt{\frac{(1-f)^2}{4f^2} + \frac{\text{control}}{f}}$
3. **PWM Mapping**: motor $\in [0,1]$ → remap to $[-1,1]$ via $v = 2m - 1$ → interpolate to $[\text{PWM\_MIN}, \text{PWM\_MAX}]$
4. **DShot Mapping**: motor $\in [0,1]$ → scale to $[\text{min\_output}, \text{max}]$ where $\text{min\_output} = \max(\text{dshot\_min}, \text{max} \cdot \text{min\_fraction})$
5. **Safety**: NaN and negative motor values clamped to 0; values > 1 clamped to 1
6. **Disarmed**: When `armed = false`, all outputs set to disarmed value (PWM: 900 µs, DShot: 0)

## Default Parameters

Based on PX4 v1.16.0 defaults:

### Attitude Controller

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| MC_ROLL_P | 6.5 | rad/s per rad | Roll P gain |
| MC_PITCH_P | 6.5 | rad/s per rad | Pitch P gain |
| MC_YAW_P | 2.8 | rad/s per rad | Yaw P gain |
| MC_YAW_WEIGHT | 0.4 | - | Yaw weight [0-1] |
| MC_ROLLRATE_MAX | 220 | °/s | Max roll rate (3.84 rad/s) |
| MC_PITCHRATE_MAX | 220 | °/s | Max pitch rate (3.84 rad/s) |
| MC_YAWRATE_MAX | 200 | °/s | Max yaw rate (3.49 rad/s) |

### Rate Controller

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| MC_ROLLRATE_P | 0.15 | - | Roll rate P gain |
| MC_ROLLRATE_I | 0.2 | - | Roll rate I gain |
| MC_ROLLRATE_D | 0.003 | - | Roll rate D gain |
| MC_ROLLRATE_K | 1.0 | - | Roll rate K multiplier |
| MC_PITCHRATE_P | 0.15 | - | Pitch rate P gain |
| MC_PITCHRATE_I | 0.2 | - | Pitch rate I gain |
| MC_PITCHRATE_D | 0.003 | - | Pitch rate D gain |
| MC_PITCHRATE_K | 1.0 | - | Pitch rate K multiplier |
| MC_YAWRATE_P | 0.2 | - | Yaw rate P gain |
| MC_YAWRATE_I | 0.1 | - | Yaw rate I gain |
| MC_YAWRATE_D | 0.0 | - | Yaw rate D gain |
| MC_YAWRATE_K | 1.0 | - | Yaw rate K multiplier |
| MC_RR_INT_LIM | 0.3 | - | Roll rate integrator limit |
| MC_PR_INT_LIM | 0.3 | - | Pitch rate integrator limit |
| MC_YR_INT_LIM | 0.3 | - | Yaw rate integrator limit |
| MC_YAW_LPF | 2.0 | Hz | Yaw torque LPF cutoff frequency |

### Control Allocator

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| MC_AIRMODE | 1 | - | Airmode: 0=disabled, 1=RP, 2=RPY |
| Geometry | Iris X | - | Quadrotor X with Iris arm lengths |
| Actuator min | 0.0 | - | Motor minimum [0, 1] |
| Actuator max | 1.0 | - | Motor maximum [0, 1] |
| Normalize RPY | true | - | Scale mix matrix for full-range RPY |

### Actuator Output

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| THR_MDL_FAC | 0.0 | - | Thrust model factor (0=linear, 0.3=racer) |
| Output Mode | PWM | - | PWM or DShot |
| PWM_MIN | 1000 | µs | PWM minimum (motor idle) |
| PWM_MAX | 2000 | µs | PWM maximum (full throttle) |
| PWM_DISARMED | 900 | µs | PWM disarmed (motors stopped) |
| DSHOT_MIN | 1 | - | DShot minimum throttle command |
| DSHOT_MAX | 1999 | - | DShot maximum throttle command |
| DSHOT_DISARMED | 0 | - | DShot disarmed value |
| DSHOT_MIN_FRAC | 0.0 | - | DShot minimum throttle fraction |

### Iris Quadrotor Geometry

| Motor | Position (X, Y) [m] | KM | Direction |
|-------|---------------------|----|-----------|
| 0 (FR) | +0.1515, +0.245 | +0.05 | CCW |
| 1 (BL) | -0.1515, -0.1875 | +0.05 | CCW |
| 2 (FL) | +0.1515, -0.245 | -0.05 | CW |
| 3 (BR) | -0.1515, +0.1875 | -0.05 | CW |

## License

BSD 3-Clause (same as PX4)

## References

- **PX4-Autopilot**: https://github.com/PX4/PX4-Autopilot
- **Algorithm Paper**: [Nonlinear Quadrocopter Attitude Control (2013)](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf)
- **PX4 Documentation**: https://docs.px4.io/

## Notes

- This extraction is for **autonomous mode only** — no manual control, VTOL, or EKF reset handling
- The matrix library is included but users can replace it with Eigen3 if preferred
- Attitude controller is stateless (proportional only); rate controller is stateful (integrators + yaw LPF)
- The rate controller's `setSaturationStatus()` method receives anti-windup feedback from the control allocator
- ✅ Phase 1 complete: Attitude control extracted and validated (3 GTests)
- ✅ Phase 2 complete: Rate control extracted, K-scaling + yaw LPF applied (16 GTests)
- ✅ Phase 3 complete: Control allocator extracted with sequential desaturation + airmode (22 GTests)
- ✅ Phase 4 complete: Thrust curve inverse + PWM/DShot output mapping (28 GTests)
- **All 69 GTests passing** — full pipeline: attitude → rate → allocator → actuator output → PWM/DShot
