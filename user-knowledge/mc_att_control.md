reference files:

* `mc_att_control.hpp`
* `mc_att_control_main.cpp`
* `mc_att_control_params.c`

## uORB publication and subsrciption

```cpp
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};			/**< rate setpoint publication */
	uORB::Publication<vehicle_attitude_setpoint_s>	_vehicle_attitude_setpoint_pub;
```

* `vehicle_attitude_setpoint` correspondent to publisher `_att_sp_pub`
* and a `vehicle_rates_setpoint` publish body rate

will load some `px4::params` 

```cpp
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLL_P>) _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>) _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>) _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>) _param_mc_yaw_weight,

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>) _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>) _param_mc_yawrate_max,

		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

		/* Stabilized mode params */
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,			/**< maximum tilt allowed for manual flight */
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,			/**< minimum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,				/**< maximum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_HOVER>)
		_param_mpc_thr_hover,			/**< throttle at which vehicle is at hover equilibrium */
		(ParamInt<px4::params::MPC_THR_CURVE>) _param_mpc_thr_curve,				/**< throttle curve behavior */

		(ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode,
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>) _param_mc_man_tilt_tau
	)
```

these params are pre-defined in `mc_att_control_params.c` , and can be revised in function `MulticopterAttitudeControl::parameters_updated()`.

## check update

Check if parameters have changed

```cpp
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}
```

when attitude is updated,

```cpp
// run controller on attitude updates
	vehicle_attitude_s v_att;

	if (_vehicle_attitude_sub.update(&v_att)) {
```

首先约束时间

```cpp
		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = v_att.timestamp_sample;
```

在下面循环中对dt有要求，若时间间隔大于20ms则限制到20ms，防止控制器相应滞后，影响控制效果。

之后检查是不是收到了更新的姿态期望值。

```cpp
		const Quatf q{v_att.q};

		// Check for new attitude setpoint
		if (_vehicle_attitude_setpoint_sub.updated()) {
			vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

			if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
			    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

				_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
				_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
				_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
			}
		}
```



