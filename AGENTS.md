# Prompts for Agents

## Overview

This repository is PX4-Autopilot 1.16.0, you can refer to the [main README](README.md) for an overview of the project.

My goal is to extract the multi-rotor attitude, rate controller from PX4 and use it as an agent in a reinforcement learning environment for a quadcopter drone.
The agent will receive observations of the drone's state and output actions of attitude setpoints (similar to the `vehicle_attitude.h`), we will use the desired attitude, thrust, and yaw rate as actions.
**The detailed goal is to realize a pure c++ implementation from attitude setpoints to the pwm outputs for the motors.** (I already got the mapping from pwm to thrust from the motor experiments)

Finally, there will be a function(api) receiving the current state and desired attitude setpoints, and output the pwm signals for the motors.

Tips:

* Use the raw attitude controller and rate controller from PX4 as much as possible.
* **IMPORTANT!** We will not take manual control into consideration, only the auto mode is considered. This will help you find the right setpoints.
* There are some existing analysis on the rate controller and PWM mixer by the user in the folder `user-knowledge`, you can refer to them for better understanding of the code. Be advised that the code version in `user-knowledge` is 1.13.0, there might be some differences with 1.16.0. (Mixer is totally decrapted in 1.16.0, so we will not use it, using control allocation instead)
* Do not edit the original code in PX4, copy the necessary files to a new folder `px4_extraction` and edit them there.

Coding style for c++:

* Follow the existing coding style in PX4 as much as possible.
* Define the class in the header file(.hpp) and implement the functions in the cpp file(.cpp). Define the class member variables in the private section as much as possible. For non-constant member variables, initialize them in the definition, do not initialize too many variables in the constructor.
* The user have Eigen3 installed in his system, you can use Eigen3 for matrix operations.
* All implemented code should be in the `px4_extraction` folder, using CMakeLists.txt to manage the build.
* We should follow C++17 standard, using modern C++ features as much as possible, but matching the existing code in PX4 is the first priority.

## The controller

### Attitude Controller

Path: `src/modules/mc_att_control/`

I will introduce the main code and name of the files for the attitude controller in PX4.

First is the attitude setpoint definition in `vehicle_attitude.h`(this is the built from message definition). Note that there is a yaw rate setpoint in the attitude setpoint, we should also mind the control design for yaw in the attitude and rate controller.

```cpp
struct vehicle_attitude_s {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float q[4];
	float delta_q_reset[4];
	uint8_t quat_reset_counter;
	uint8_t _padding0[7]; // required for logger


#ifdef __cplusplus
	static constexpr uint32_t MESSAGE_VERSION = 0;

#endif
};
```
The main attitude controller api is in `mc_att_control.hpp` and `mc_att_control_main.cpp`, the parameters are defined in `mc_att_control_params.c`.

The control algorithm is in `AttitudeControl.cpp` and `AttitudeControl.hpp`.

### Rate Controller

Path: `src/modules/mc_rate_control/`

Ahead of all, refer `user-knowledge/mc_rate_control.md`  for better understanding of the rate controller in PX4 1.13.0, which may help you a lot.

The main rate controller api is in `MulticopterRateControl.hpp` and `MulticopterRateControl.cpp`.
The parameters are defined in `mc_rate_control_params.c`.


### Control Allocation

Path: `src/modules/control_allocator/` and `src/lib/control_allocation/`

Ahead of all, refer `user-knowledge/control_allocation_mixer.md` for better understanding of the control allocation in PX4 1.13.0, which may help you a lot.

* The main control allocation api is in `ControlAllocator.hpp` and `ControlAllocator.cpp`.
* There are lots of control effectiveness models defined in `VehicleActuatorEffectiveness` folder, for multi-rotor drones, we will use `ActuatorEffectivenessMultirotor.hpp` and `ActuatorEffectivenessMultirotor.cpp`. They are derived from the base class `ActuatorEffectiveness.hpp` and `ActuatorEffectiveness.cpp` in `src/lib/control_allocation/actuator_effectiveness/`.
* The main control allocation algorithm is `src/lib/control_allocation/control_allocation/`.
