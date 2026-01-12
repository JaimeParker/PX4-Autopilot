# Multicopter Rate Control

* PX4 1.13.0 [github link 1.13.0](https://github.com/PX4/PX4-Autopilot/tree/v1.13.0)
* 1.13.0 **mc_rate_control** root [link](https://github.com/PX4/PX4-Autopilot/tree/v1.13.0/src/modules/mc_rate_control)

## rate setpoint

which is `_rate_sp` in code, the source of it depends on mode

* for **Manual control in ACRO mode**, it's related to stick input (remote controller)
* for **other mode**, it's from the `vehicle_rates_setpoint` topic

If not in manual ACRO mode, the rate setpoint comes from the `vehicle_rates_setpoint` topic, where values are generally populated by an attitude controller running upstream. The received values are checked for validity and applied directly to `_rates_sp`.

For using `mavros_msgs::AttitudeTarget` and set **mask type=rate only**, it's just the ros topic.

```c++
if (_v_control_mode.flag_control_manual_enabled && !_v_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			// pass
		} else {
			// use rates setpoint topic
			vehicle_rates_setpoint_s v_rates_sp;

			if (_v_rates_sp_sub.update(&v_rates_sp)) {
				_rates_sp(0) = PX4_ISFINITE(v_rates_sp.roll)  ? v_rates_sp.roll  : rates(0);
				_rates_sp(1) = PX4_ISFINITE(v_rates_sp.pitch) ? v_rates_sp.pitch : rates(1);
				_rates_sp(2) = PX4_ISFINITE(v_rates_sp.yaw)   ? v_rates_sp.yaw   : rates(2);
				_thrust_sp = -v_rates_sp.thrust_body[2];
			}
		}
```

if not manual control and attitude control disabled, rate control will handle.

注意此处给拿来的thrust取了个负值，这和飞控日志反映出来的是一致的。

## thrust setpoint

you might wonder about the thrust, since rate cannot control the drone fully, it's the same as rate setpoint.

* In ACRO mode, the thrust is directly set from the `manual_control_setpoint.z` input, constrained between `0.0` and `1.0` in the range `[0, 1]`.
* **In Other Modes**: The thrust setpoint comes from `vehicle_rates_setpoint.thrust_body[2]`. Since thrust is generally expressed as a percentage of the total thrust capacity, it is stored in `_thrust_sp` as a normalized value in `[0, 1]`, where `1.0` represents maximum thrust.

## update (controller)

## output to mixer

let's see the answer of chatgpt:

The outputs of this rate control class that are sent to the **mixer** are the:

- **Torque (Actuator Controls 0-2)**: These are the torque requirements computed by the PID controller based on the difference between actual and desired rates. They are passed as roll, pitch, and yaw control values in the `actuator_controls_s` message.
- **Thrust (Actuator Control 3)**: The normalized thrust is passed to `actuator_controls[3]` as `_thrust_sp`, which is also in `[0, 1]`.

注意，在此处求解出的 torque 应该未做限幅处理，直接被发送到了 actuator control 话题，这和日志中的情况也能对应上（大部分处于-1，1之间，极少部分abs>1）

These outputs (`actuator_controls[0-3]`) are sent to the mixer. The mixer then calculates individual motor outputs based on the vehicle geometry, taking these torque and thrust requirements and mapping them to PWM outputs or motor speeds.
