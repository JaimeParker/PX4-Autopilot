# AI Agent Instructions for PX4 Controller Extraction

## 1. Project Background and Goal
This project extracts the core inner-loop control and allocation math from the PX4 Autopilot source code to create a standalone, lightweight C++ API.

**The Ultimate Objective:** Provide a purely deterministic, fast-executing C++ dynamics/control backend for Reinforcement Learning (RL) training of agile quadrotors.
* **The RL Policy outputs:** Attitude/Thrust commands OR Body Rate/Thrust commands.
* **This Extracted Module outputs:** PWM or normalized actuator commands.
* **The Physics Engine (User's Domain) handles:** Motor parameters $\rightarrow$ Force/Torque generation $\rightarrow$ Rigid body state updates.

To achieve sim-to-real transfer seamlessly, this code must exactly replicate the math of PX4's `mc_att_control`, `mc_rate_control`, and `control_allocator`, but **without any of PX4's middleware overhead**.

## 2. Strict Coding Directives

As an AI assisting in this repository, you must adhere to the following absolute rules when generating or refactoring code:

* **Rule 1: NO uORB.** You are strictly forbidden from including or using `uORB` publishers, subscribers, or topic headers. All data flow must be handled via standard C++ structs passed by reference or pointer.
* **Rule 2: NO PX4 Middleware.** Strip out all dependencies on the PX4 Parameter system (`param_get`, `DEFINE_PARAMETERS`), Work Queues (`px4_work_queue`), High-Resolution Timers (`hrt_absolute_time`), and Perf counters.
* **Rule 3: Reuse Clean PX4 Libraries Directly.** You are heavily encouraged to copy necessary, dependency-free PX4 libraries (like `mathlib`, `matrix`, filters, or standalone math utilities) directly into this project's folder structure. Use them directly without rewriting, provided they do not violate Rules 1 and 2. Do not introduce external math dependencies like Eigen unless explicitly instructed.
* **Rule 4: Time Handling.** Time is no longer fetched from the OS or hardware. Every update loop function (e.g., `update(dt)`) must accept the time step `dt` (in seconds) as a standard float/double argument provided by the external RL simulation environment.
* **Rule 5: Parameter Handling.** PX4 parameters must be converted into standard C++ structs (e.g., `RateControlParams`, `AttitudeControlParams`). The external simulation will instantiate these structs and pass them into the controller classes.
* **Rule 6: Quadrotor Only.** Drop any code related to Fixed-Wing, VTOL, Rovers, or Submersibles. We are solely concerned with multirotor (specifically quadrotor)  dynamics, kinematics and control.
* **Rule 7: C++17 Standard.** Utilize modern C++17 features for safety and clarity, but avoid heavy standard library allocations inside the control loop to maintain high execution speed.

## 3. Current Repository State
* The `attitude_control` module is successfully extracted.
* `lib/matrix` and `lib/mathlib` are present and integrated via CMake.
* The project builds as a standalone C++ library (`px4_attitude_controller_extraction`).

## 4. Execution Roadmap

You will assist the human developer in executing the following three phases sequentially. Do not jump to a later phase until the current one is validated.

### Phase 1: Attitude Control Validation
**Objective:** Ensure the extracted `AttitudeControl` class performs identical mathematical operations to the original PX4 implementation.
* **Agent Task:** Develop robust unit tests in `test/AttitudeControlTest.cpp`.
* **Test Design:** 1. Initialize the controller with standard quadrotor tuning parameters.
    2. Feed deterministic inputs: a specific target quaternion and a specific current vehicle attitude quaternion.
    3. Assert that the output `vehicle_rates_setpoint` matches expected analytical results. Include edge cases (e.g., 180-degree yaw errors, pitch lock).

### Phase 2: Extract Rate Control (`mc_rate_control`)
**Objective:** Port the body rate PID controllers.
* **Agent Task:** Extract the core logic from PX4's `src/modules/mc_rate_control/RateControl`. Copy any necessary, standalone discrete-time filters from PX4's `mathlib` directly into our `lib` folder to support this.
* **Inputs (Simulated):** Target body rates (from Phase 1), Current actual body rates (from RL simulation gyro), `dt`.
* **Outputs:** Normalized torque setpoints (3D vector: roll, pitch, yaw).
* **Refactoring:** Create a `RateControlParams` struct for PID gains ($K_p, K_i, K_d$, feedforward, and rate limits).

### Phase 3: Extract Control Allocation (The Critical Step)
**Objective:** Map normalized torques and thrust to individual motor PWM/actuator commands.
* **Context:** PX4 uses a dynamic control allocator. We must strip away the dynamic loading and matrix inversion overhead and implement the static matrix multiplication for a standard quadrotor.
* **Agent Task:** Reconstruct the multirotor effectiveness matrix application.
* **Inputs:** Normalized torque setpoint (from Phase 2), Normalized thrust setpoint (from RL policy or altitude controller).
* **Outputs:** Actuator commands (4-element array mapping to Motors 1-4).
* **Key Implementations:**
    1.  **Geometry Matrix:** Define a strict geometry struct (e.g., Quadrotor X) to compute the effectiveness matrix (roll, pitch, yaw, thrust factors per motor).
    2.  **Mixer Logic:** Implement the desaturation logic.
    3.  **Actuator Mapping:** Implement the final thrust curve mapping.

## 5. Standard API Interface Design Pattern
When generating code for any module, follow this purely data-driven C++ pattern:

```cpp
// 1. Parameter Struct (Replaces PX4 Param System)
struct RateControlParams {
    matrix::Vector3f gain_p;
    matrix::Vector3f gain_i;
    matrix::Vector3f gain_d;
    // ... limits and anti-windup
};

// 2. Input/Output Structs (Replaces uORB)
struct RateControlInput {
    matrix::Vector3f rates_sp;
    matrix::Vector3f rates_actual;
    float dt;
};

struct RateControlOutput {
    matrix::Vector3f torque_sp;
};

// 3. Class Definition
class RateControl {
public:
    void setParams(const RateControlParams& params);
    void update(const RateControlInput& input, RateControlOutput& output);
private:
    // Internal states (integrators, prev errors)
};
```
