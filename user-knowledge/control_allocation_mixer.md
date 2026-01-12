# Control Allocation and Mixer in PX4

[author](https://github.com/JaimeParker)，适用于 **PX4 1.13.0**

![](images/mixing_overview.DPPpqncZ.png)

PX4 有两种控制分配方式，分别是：

* mixer, in [PX4/src/lib/mixer](https://github.com/PX4/PX4-Autopilot/tree/v1.13.0/src/lib/mixer), 在新版PX4中已经被移除
* Control Allocation, in [PX4/src/modules/control_allocator](https://github.com/PX4/PX4-Autopilot/tree/v1.13.0/src/modules/control_allocator)

> **INFO**
>
> Control allocation replaces the legacy mixing approach used in PX4 v1.13 and earlier. For PX4 v1.13 documentation see: [Mixing & Actuators](https://docs.px4.io/v1.13/en/concept/mixing.html), [Geometry Files](https://docs.px4.io/v1.13/en/concept/geometry_files.html) and [Adding a New Airframe Configuration](https://docs.px4.io/v1.13/en/dev_airframes/adding_a_new_frame.html).

PX4 有一个参数 **`SYS_CTRL_ALLOC`** 可以决定是否打开 Control allocator，如果打开，则控制分配走新的Control Allocation，否则就 fall back to 老的 mixer。

但很奇怪，我没在 PX4 的 [parameter reference](https://docs.px4.io/main/en/advanced_config/parameter_reference.html) 里找到这个参数，猜测是 PX4 把 mixer 废弃之后，这个参数改成默认启动的了，所以不需要设置 enable or disable，但是 1.13.0 版本的 PX4， 使用 `iris_alloc` model, 仍然可以在QGC中找到这个参数（默认是关闭的），描述如下：

> If disabled, the existing mixing implementation is used. If enabled, dynamic control allocation with runtime configuration of the mixing and output functions is used.
> Note: this is work-in-progress and not all vehicle types are supported yet.

对于2024.11月官方doc：

* [Development/Concepts/Control Allocation](https://docs.px4.io/main/en/concept/control_allocation.html)

## 1. control allocation

control_allocator 目录：

* `CMakeList.txt`
* `ControlAllocator`
* `module.yaml`
* `ControlAllocation`
  * `CMakeList.txt`
  * `ControlAllocation`
  * others
* `ActuatorEffectiveness`

注意，1.13.0的control_allocator还是依赖mixer的：

```cmake
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(ActuatorEffectiveness)
add_subdirectory(ControlAllocation)

px4_add_module(
	MODULE modules__control_allocator
	MAIN control_allocator
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	STACK_MAIN
		3000
	SRCS
		ControlAllocator.cpp
		ControlAllocator.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		mathlib
		ActuatorEffectiveness
		ControlAllocation
		mixer
		px4_work_queue
)
```

### 1.1 control allocator class

关于control_allocator，PX4 doc 在 [modules/modules_controller](https://docs.px4.io/main/en/modules/modules_controller.html#control-allocator) 定义如下：

> Source: [modules/control_allocator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/control_allocator)
>
> **Description**: This implements control allocation. It takes torque and thrust setpoints as inputs and outputs actuator setpoint messages.

![images](images/ctrl_alloc.png)

#### 1.1.1 parameters_updated() in Run()

Run 函数中的参数加载（更新）

```c++
	// Check if parameters have changed
	if (_parameter_update_sub.updated() && !_armed) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}
```

着重分析 parameters_updated，总共涉及了 4 个函数：

```c++
void ControlAllocator::parameters_updated() {
	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}

	// Allocation method & effectiveness source
	// Do this first: in case a new method is loaded, it will be configured below
	bool updated = update_effectiveness_source();
	update_allocation_method(updated); // must be called after update_effectiveness_source()

	if (_num_control_allocation == 0) {
		return;
	}

	for (int i = 0; i < _num_control_allocation; ++i) {
		_control_allocation[i]->updateParameters();
	}

	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}
```

##### update_effectiveness_source

根据参数 **[CA_AIRFRAME](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#CA_AIRFRAME)**, 完成类成员变量 `_actuator_effectiveness` 的实例化。

对于多旋翼，会使用 `ActuatorEffectiveness` 的子类 [`ActuatorEffectivenessMultirotor`](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessMultirotor.cpp) 来完成实例化。

```c++
		// try to instanciate new effectiveness source
		ActuatorEffectiveness *tmp = nullptr;

		switch (source) {
		case EffectivenessSource::NONE:
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;
```

##### update_allocation_method()

```c++
void ControlAllocator::update_allocation_method(bool force) {
	// 根据参数 CA_METHOD 决定使用的分配方法
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();

	if (_allocation_method_id != configured_method || force) {
		// 对于多旋翼，维度为6*1，6个控制量，1个矩阵
		matrix::Vector<float, NUM_ACTUATORS> actuator_sp[ActuatorEffectiveness::MAX_NUM_MATRICES];

		// 对于多旋翼，只有一个矩阵
		_num_control_allocation = _actuator_effectiveness->numMatrices();

		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		// 对于多旋翼，const normalize[0] = true;
		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);
        // normalize_rpq[0] = true
```

`getNormalizeRPY` 在multirotor 中的实现如下：

```c++
	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}
```

结合上面的 `desired_methods` 变量，下面研究一下 **[CA_METHOD](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#CA_METHOD)** 参数为 AUTO 时对多旋翼的影响（AUTO即代表使用PX4定义的默认值）

```c++
		// 定义对于不同矩阵的分配方法，对于多旋翼，只有一个矩阵
		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		// 对于多旋翼，ActuatorEffectivenessMultirotor 中定义固定使用 SEQUENTIAL_DESATURATION 方法
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		// 对于多旋翼，const normalize[0] = true;
		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);

		for (int i = 0; i < _num_control_allocation; ++i) {
			AllocationMethod method = configured_method;

			if (configured_method == AllocationMethod::AUTO) {
				method = desired_methods[i];
			}

			switch (method) {
			case AllocationMethod::PSEUDO_INVERSE:
				_control_allocation[i] = new ControlAllocationPseudoInverse();
				break;

			case AllocationMethod::SEQUENTIAL_DESATURATION:
				_control_allocation[i] = new ControlAllocationSequentialDesaturation();
				break;
```

那么可以确定，**对于多旋翼，AUTO即使用序列去饱和方法**。see [getDesiredAllocationMethod of ActuatorEffectivenessMultirotor](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessMultirotor.hpp#L47).

最后是两个设置：

```c++
			if (_control_allocation[i] == nullptr) {
				PX4_ERR("alloc failed");
				_num_control_allocation = 0;

			} else {
				// _control_allocation[0]->_normalize_rpy = normalize_rpy[0] = true for multirotor
				_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);
				// 对 control_allocation[i] 中的 _actuator_sp 赋值并 clip
				_control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);
			}
```

##### updateParameters()

see [ControlAllocation.hpp](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocation/ControlAllocation.hpp#L216), virtual function. 此函数只在序列去饱和方法中被重载，则若 **CA_METHOD=伪逆**，此行代码无效。

TODO：序列去饱和方法中的实现

##### update_effectiveness_matrix_if_needed()

这个函数在整个代码中被调用了两次，一次是在当前的参数更新阶段（我估计只会调用一次），另一次是出现在 Run 函数的 `do_update` 判断中，预计会被多次调用。

但在 Run 函数中，由于参数设置的是

在函数 parameters_update() 中，update_effectiveness_matrix_if_needed() 的参数为 `EffectivenessUpdateReason::CONFIGURATION_UPDATE`，也就是 enum 值 1, see:

```c++
enum class EffectivenessUpdateReason {
	NO_EXTERNAL_UPDATE = 0,
	CONFIGURATION_UPDATE = 1,
	MOTOR_ACTIVATION_UPDATE = 2,
};
```

简单概括一下函数结构，

```c++
void ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason) {
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	// 对于多旋翼，此condition只有在reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE时成立
	// 即在 Run 函数中，即使每次 do_update 为 true，也不会执行此 if segment，相当于函数跳过
	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
        // ...
    }
```

所以对于多旋翼，只在参数更新阶段起了作用。

当 reason = 1时，在多旋翼的actuator子类中，首先执行 getEffectivenessMatrix 函数，详见 **1.3.2**

总之，对于四旋翼，经过getEffectivenessMatrix函数后，对`config.effectiveness_matrices[0]`完成了赋值，获得了对应机型的控制效率矩阵。

且除此之外，`config`结构体的**其他成员均为默认值**。

```c++
				// 对于多旋翼，selected_matrix = 0，即只有一个矩阵，这是config.matrix_selection_indexes中的默认值
				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				// for quadrotor, the actuators are motors
				if ((ActuatorType)actuator_type == ActuatorType::MOTORS) {
					if (actuator_type_idx >= MAX_NUM_MOTORS) {
						PX4_ERR("Too many motors");
						_num_actuators[actuator_type] = 0;
						break;
					}

					if (_param_r_rev.get() & (1u << actuator_type_idx)) {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;

					} else {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 0.f;
					}
```

如果电机能够反转，则最小值设为-1，否则0；根据iris_ctrlalloc status出来的，是0。

```c++
		for (int i = 0; i < _num_control_allocation; ++i) {
            // 设置最大值1，最小值0，斜率限制（默认无限制）
			_control_allocation[i]->setActuatorMin(minimum[i]);
			_control_allocation[i]->setActuatorMax(maximum[i]);
			_control_allocation[i]->setSlewRateLimit(slew_rate[i]);

			// Set all the elements of a row to 0 if that row has weak authority.
			// That ensures that the algorithm doesn't try to control axes with only marginal control authority,
			// which in turn would degrade the control of the main axes that actually should and can be controlled.

			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[i];
			
            // 对控制分配矩阵限制，若一行都是很小的值，就设为0，防止干扰
			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;

				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) {
						all_entries_small = false;
					}
				}

				if (all_entries_small) {
					matrix.row(n) = 0.f;
				}
			}

			// Assign control effectiveness matrix
			int total_num_actuators = config.num_actuators_matrix[i];
            // 对于四旋翼，actuator 4个，控制矩阵有效，其他默认0，执行set函数
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
		}

		trims.timestamp = hrt_absolute_time();
		_actuator_servos_trim_pub.publish(trims);  // 未对trims做处理，默认0元素矩阵
```

在这之后，更新控制矩阵参数的部分就结束了，接下来进入 setEffectivenessMatrix 函数。

基类中此函数的声明定义见 1.2.1，派生类（我用的AEMc，后文缩写AEM）见1.3.3。

总结，对于伪逆ControlAllocation类中的：

```c++
	// Allocate
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
```

* `matrix::Vector<float, NUM_AXES> _control_trim;`，`NUM_AXES=6`
* `matrix::Vector<float, NUM_ACTUATORS> _actuator_trim; `，`NUM_ACTUATORS=16`
* 总之全是0，可以忽略（仅对于MC）

#### 1.1.2 Run() main function

在 [ControlAllocator.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocator.cpp) 的 `Run` 函数中，首先接收了 RateControl 发来的 torque and thrust setpoint:

```c++
	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	// Also run allocator on thrust setpoint changes if the torque setpoint
	// has not been updated for more than 5ms
	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);

		if (dt > 5_ms) {
			do_update = true;
			_timestamp_sample = vehicle_thrust_setpoint.timestamp_sample;
		}
	}
```

注意还对 thrust 的更新做了一个间隔 dt 设置，小于 5ms 的新 thrust setpoint 则不会造成更新。

之后是主要的更新部分：

* 对于普通的四旋翼，`_num_control_allocation = 1`
* 剩下的就是 control allocation 类的部分了 

```c++
		for (int i = 0; i < _num_control_allocation; ++i) {

			_control_allocation[i]->setControlSetpoint(c[i]);

			// Do allocation
			_control_allocation[i]->allocate();
			_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp);

			if (_has_slew_rate) {
				_control_allocation[i]->applySlewRateLimit(dt);
			}

			_control_allocation[i]->clipActuatorSetpoint();
		}
```

- 对于四旋翼，首先设置了一个 [0, 6] 的 desired torque (physical value) and thrust ([0, 1], z axis non zero)
- 之后调用`allocate()` 函数，**see 1.2.2**

仅对伪逆法分析，所以此时已经获得了关键的 **`_actuator_sp`**

之后调用

```c++
			// seems did nothing for multirotor
			_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp);
```

但此基类方法没有在MC派生类实现，因此跳过。

```c++
			_control_allocation[i]->clipActuatorSetpoint();
```

又做了一次clip。对于四旋翼，就是 $[0, 1]$ , see 1.1.1 update_effectiveness_matrix_if_needed()

到了最后的发布函数，**publish_actuator_controls**

##### publish_actuator_controls()

```c++
	actuator_motors_s actuator_motors;
	actuator_motors.control[motors_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
	_actuator_motors_pub.publish(actuator_motors);
```

可以简单认为上述计算出来的actuator_sp直接拿来发布了。

pub的定义为：

```c++
uORB::Publication<actuator_motors_s>	_actuator_motors_pub{ORB_ID(actuator_motors)};
```

此话题可能对应日志中的`actuator_controls.control[0-3]`。

**至此，发布完成，剩下交给接收端，处理此信息并转化为pwm值**。







-------

##### slew and `applySlewRateLimit()`

在 `parameters_updated()` 函数中，会决定 `_has_slew_true` 的值：

```c++
	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}
```

这是与电机和伺服的定义有关的，**如果定义的值=0**，则  `_has_slew_true` 保持 false。

这些值一般定义在 [module.yaml](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/module.yaml) ，比如：

```yaml
        # Motor params
        CA_R${i}_SLEW:
            description:
                short: Motor ${i} slew rate limit
                long: |
                  Minimum time allowed for the motor input signal to pass through
                  the full output range. A value x means that the motor signal
                  can only go from 0 to 1 in minimum x seconds (in case of
                  reversible motors, the range is -1 to 1).

                  Zero means that slew rate limiting is disabled.
            type: float
            decimal: 2
            increment: 0.01
            num_instances: *max_num_mc_motors
            min: 0
            max: 10
            default: 0.0

        # Servo params
        CA_SV${i}_SLEW:
            description:
                short: Servo ${i} slew rate limit
                long: |
                  Minimum time allowed for the servo input signal to pass through
                  the full output range. A value x means that the servo signal
                  can only go from -1 to 1 in minimum x seconds.

                  Zero means that slew rate limiting is disabled.
            type: float
            decimal: 2
            increment: 0.05
            num_instances: *max_num_servos
            min: 0
            max: 10
            default: 0.0
```

TODO: applySlewRateLimit(dt)

##### clipActuatorSetpoint()

详见 control allocation class

### 1.2 control allocation

* **base** class: [`ControlAllocation`](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocation/ControlAllocation.cpp)

* **derived** class: [`ControlAllocationPseudoInverse`](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocation/ControlAllocationPseudoInverse.cpp) and [`ControlAllocationSequentialDesaturation`](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocation/ControlAllocationSequentialDesaturation.cpp)，伪逆矩阵和序列去饱和方法。

具体选用哪种分配方法，由参数 **[CA_METHOD](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#CA_METHOD)** 决定。（可能默认是automatic）

> **For sequential desaturation**, it processes each control input individually, prioritizing primary controls (like yaw, pitch, and roll) and ensuring that actuator limits are respected. If any control request exceeds actuator limits, the method desaturates it, aiming to retain the primary control effectiveness as much as possible.
>
> **For pseudo-inverse**, it calculates the actuator commands using a mathematical pseudo-inverse of the control effectiveness matrix. This approach evenly distributes control among the actuators to achieve the desired torque and thrust as closely as possible, without prioritizing any particular control input.
>
> from chatgpt-4o

#### 1.2.1 base class `ControlAllocation`

注意构造函数中，

```c++
ControlAllocation::ControlAllocation()
{
	_control_allocation_scale.setAll(1.f);
	_actuator_min.setAll(0.f);
	_actuator_max.setAll(1.f);
}
```

##### setActuatorSetpoint()

在 `setActuatorSetpoint` 函数中，主要是赋值和clip，现在所有的 actuator control 的值都在 $[0, 1]$ 之间。

```c++
void
ControlAllocation::setActuatorSetpoint(
	const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator_sp)
{
	// Set actuator setpoint
	_actuator_sp = actuator_sp;

	// Clip
	clipActuatorSetpoint(_actuator_sp);
}

void
ControlAllocation::clipActuatorSetpoint(matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator) const
{
	for (int i = 0; i < _num_actuators; i++) {
		if (_actuator_max(i) < _actuator_min(i)) {
			actuator(i) = _actuator_trim(i);

		} else if (actuator(i) < _actuator_min(i)) {
			actuator(i) = _actuator_min(i);

		} else if (actuator(i) > _actuator_max(i)) {
			actuator(i) = _actuator_max(i);
		}
	}
}
```

除此之外，在 base class 中还有一个名字很明显但可能还是test的函数 **normalizeActuatorSetpoint**，虽然他真的很像去做归一化的。（只在header declare，在cpp define了）

![](images/norm.png)

##### setEffectivenessMatrix()

用于设置控制效率矩阵，对于四旋翼是根据 [PX4/ROMFS/px4_fmu_common/init.d/airframes](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/ROMFS/px4fmu_common/init.d/airframes/) 或者 [PX4/src/lib/mixer/MultirotorMixer/geometries](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/lib/mixer/MultirotorMixer/geometries/quad_wide.toml)。

基类声明是：

```c++
	/**
	 * Set the control effectiveness matrix
	 *
	 * @param B Effectiveness matrix
	 */
	virtual void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
					    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
					    bool update_normalization_scale);
```

基类定义是：

```c++
void ControlAllocation::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	_effectiveness = effectiveness;
	ActuatorVector linearization_point_clipped = linearization_point;
	clipActuatorSetpoint(linearization_point_clipped);
	_actuator_trim = actuator_trim + linearization_point_clipped;
	clipActuatorSetpoint(_actuator_trim);
	_num_actuators = num_actuators;
	_control_trim = _effectiveness * linearization_point_clipped;
}
```

基本上就是一个赋值+clip的操作，所以从这里看不出数据来源，只能找此函数在基类中的具体实现，以及在上层文件中对` ControlAllocation` 的实例化操作。

此函数最顶层的调用出现在 [ControlAllocator.cpp Line 529](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocator.cpp#L529), in function **update_effectiveness_matrix_if_needed**()：

```c++
// Assign control effectiveness matrix
int total_num_actuators = config.num_actuators_matrix[i];
_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i], config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
```

所以所有的数据都来自此 `config` ，而该对象被实例化在 [Line 429](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocator.cpp#L429)，是函数`ControlAllocator::update_effectiveness_matrix_if_needed`的局部变量，其被赋值于：

```c++
void ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason) {
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
		_last_effectiveness_update = hrt_absolute_time();
```

那么关键就是 `update_effectiveness_matrix_if_needed` 在哪里被调用了，提供的具体参数是什么。

在整个PX4 1.13.0 目录中，只有两处调用，均位于  [`ControlAllocator.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocator.cpp) ， Line 134 和 Line 375：

```c++
void ControlAllocator::parameters_updated() {
	// Line 134
	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

void ControlAllocator::Run() {
	if (do_update) {
		_last_run = now;
		// Line 375
		update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::NO_EXTERNAL_UPDATE); 
}
```

而 `reason` 也只是一个枚举类型，

```c++
enum class EffectivenessUpdateReason {
	NO_EXTERNAL_UPDATE = 0,
	CONFIGURATION_UPDATE = 1,
	MOTOR_ACTIVATION_UPDATE = 2,
};
```

所以 `config` 被赋值正确应该是在 `ControlAllocator::update_effectiveness_matrix_if_needed` 中调用 `ActuatorEffectiveness::getEffectivenessMatrix()` 这一函数，

```c++
	/**
	 * Get the control effectiveness matrix if updated
	 *
	 * @return true if updated and matrix is set
	 */
	virtual bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) { return false;}
```

根据函数声明可以看出，对 `config` 的引用是对其赋值的关键。

**后续见 `ActuatorEffectiveness` 类**。

#### 1.2.2 class [`ControlAllocationSequentialDesaturation`](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocation/ControlAllocationSequentialDesaturation.cpp)

##### **allocate()**

```c++
void ControlAllocationSequentialDesaturation::allocate() {
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	switch (_param_mc_airmode.get()) {
	case 1:
		mixAirmodeRP();
		break;

	case 2:
		mixAirmodeRPY();
		break;

	default:
		mixAirmodeDisabled();
		break;
	}
}
```

其中，`updatePseudoInverse()` 定义在 PseudoInverse 子类中。但是根据注释，可能一般的四旋翼不用更新。

之后就是根据不同的 **_param_mc_airmode** 参数，决定怎么融。此参数来自 **[MC_AIRMODE](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#MC_AIRMODE)**:

> Multicopter air-mode.
>
> The air-mode enables the mixer to increase the total thrust of the multirotor in order to keep attitude and rate control even at low and high throttle. This function should be disabled during tuning as it will help the controller to diverge if the closed-loop is unstable (i.e. the vehicle is not tuned yet). Enabling air-mode for yaw requires the use of an arming switch.
>
> - `0`: Disabled
> - `1`: Roll/Pitch
> - `2`: Roll/Pitch/Yaw
>
> default is 0

##### **mixAirmodeDisabled()**

TODO

#### 1.2.3 class [`ControlAllocationPseudoInverse`](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocation/ControlAllocationPseudoInverse.cpp)

##### **allocate()**

```c++
void ControlAllocationPseudoInverse::allocate(){
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	// Allocate
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
}
```

其中最关键的公式和一般的控制分配（比如全权的）有些出入，主要在：

* `_actuator_trim` 具体的值，MC是0
* `_control_trim` 具体的值，MC是0
* `_mix` 的 normalize 与否

要了解 `_mix` 的值，需要先关注函数`ControlAllocationPseudoInverse::updatePseudoInverse()` ，这是伪逆和序列去饱和两种方法都要调用的。

##### updatePseudoInverse()

无论使用哪种分配方法，这个函数都是一定会被调用的。

```c++
ControlAllocationPseudoInverse::updatePseudoInverse()
{
	if (_mix_update_needed) {
		matrix::geninv(_effectiveness, _mix);

		if (_normalization_needs_update) {
			updateControlAllocationMatrixScale();
			_normalization_needs_update = false;
		}

		normalizeControlAllocationMatrix();
		_mix_update_needed = false;
	}
}
```

> The function starts by calling `updatePseudoInverse()`. This step ensures that the matrix `_mix`, which translates control setpoints to actuator commands, is up-to-date.
>
> If `_mix` was previously marked as needing an update (for example, if the effectiveness matrix changed), `updatePseudoInverse()` recalculates it. This step ensures the allocation process uses the most current transformation matrix.
>
> from chatgpt-4o

关于 `_mix_update_needed` 这个量的 bool，我估计一开始应该是true，更新之后会置 false，之后无特殊情况不更新。（control allocator 的介绍里有 dynamic control allocator 几个词，估计是和 dynamic 有关，但是普通的四旋翼应该用不到）

此时已经获得了控制效能矩阵，并对一些小值做了处理，**see 1.2.1 setEffectivenessMatrix()**

根据`ControlAllocationPseudoInverse::setEffectivenessMatrix`可知，`_mix_update_needed = true;`，所以需要执行下列步骤，**对控制效能矩阵其求逆获得 `_mix`**

另一个flag的值需要看`update_effectiveness_matrix_if_needed()`函数中是怎么调用`setEffectivenessMatrix()`:

```c++
_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
                    config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
```

结合 ControlAllocator.cpp parameters_update() 函数中调用的`update_effectiveness_matrix_if_needed()`，只有这次是需要进行归一化的，也就是此时 `_normalization_needs_update=true`，执行**updateControlAllocationMatrixScale**()

获得了归一化尺度。

之后执行归一化，call normalizeControlAllocationMatrix()

##### **updateControlAllocationMatrixScale**()

see **1.1.1** update_allocation_method()，`_normalize_rpy=true`

之后涉及对rpy力矩和推力的归一化，see [ControlAllocationPseudoInverse Line 76](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocation/ControlAllocationPseudoInverse.cpp#L76)

##### normalizeControlAllocationMatrix()

比较简单，直接放了

```c++
	if (_control_allocation_scale(0) > FLT_EPSILON) {
		_mix.col(0) /= _control_allocation_scale(0);
		_mix.col(1) /= _control_allocation_scale(1);
	}

	if (_control_allocation_scale(2) > FLT_EPSILON) {
		_mix.col(2) /= _control_allocation_scale(2);
	}

	if (_control_allocation_scale(3) > FLT_EPSILON) {
		_mix.col(3) /= _control_allocation_scale(3);
		_mix.col(4) /= _control_allocation_scale(4);
		_mix.col(5) /= _control_allocation_scale(5);
	}

	// Set all the small elements to 0 to avoid issues
	// in the control allocation algorithms
	for (int i = 0; i < _num_actuators; i++) {
		for (int j = 0; j < NUM_AXES; j++) {
			if (fabsf(_mix(i, j)) < 1e-3f) {
				_mix(i, j) = 0.f;
			}
		}
	}
```

最后对`_mix`矩阵中的小值做了处理，防止误差。

##### setEffectivenessMatrix()

```c++
void
ControlAllocationPseudoInverse::setEffectivenessMatrix(
    const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
    bool update_normalization_scale)
{
    ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
            update_normalization_scale);
    _mix_update_needed = true;
    _normalization_needs_update = update_normalization_scale;
}
```

### 1.3 actuator effectiveness

#### 1.3.1 base class [`ActuatorEffectiveness`](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectiveness.cpp)

##### getEffectivenessMatrix()

```c++
virtual bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) { return false;}
```

需要看在具体的子类中是如何实现重载的。

#### 1.3.2 derived class [`ActuatorEffectivenessRotors`](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessRotors.cpp)

##### constructor

声明为：

```c++
ActuatorEffectivenessRotors(ModuleParams *parent, AxisConfiguration axis_config = AxisConfiguration::Configurable,
				    bool tilt_support = false);
```

构造函数定义如下：

```c++
ActuatorEffectivenessRotors::ActuatorEffectivenessRotors(ModuleParams *parent, AxisConfiguration axis_config,
		bool tilt_support)
	: ModuleParams(parent), _axis_config(axis_config), _tilt_support(tilt_support)
{
	for (int i = 0; i < NUM_ROTORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);

		if (_axis_config == AxisConfiguration::Configurable) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AX", i);
			_param_handles[i].axis_x = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AY", i);
			_param_handles[i].axis_y = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AZ", i);
			_param_handles[i].axis_z = param_find(buffer);
		}

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
		_param_handles[i].moment_ratio = param_find(buffer);

		if (_tilt_support) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_TILT", i);
			_param_handles[i].tilt_index = param_find(buffer);
		}
	}

	_count_handle = param_find("CA_ROTOR_COUNT");

	updateParams();
}
```

很明显，都是 [module.yaml](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/module.yaml) 里的值。

**所以我认为在使用`control_allocator`（而不是mixer）时，加载的参数应该来自[PX4/ROMFS/px4_fmu_common/init.d/airframes](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/ROMFS/px4fmu_common/init.d/airframes/)**

构造函数的作用很明显了，读取了电机位置参数，轴的参数（对于普通多旋翼估计是NED的，也有可能是机体FLU，待定）

**轴的参数**有如下几种：

```c++
	enum class AxisConfiguration {
		Configurable, ///< axis can be configured
		FixedForward, ///< axis is fixed, pointing forwards (positive X)
		FixedUpwards, ///< axis is fixed, pointing upwards (negative Z)
	};
```

如果按照 module.yaml 中的默认定义，是 $[0, 0, -1]$，并且AEM在实例化AER时也为对此参数赋值，按默认处理。

**至于tilt**，按照声明中的默认参数，一般是false，在MC的实例化中（见1.3.3），也未给定参数，因此false。（MC本来也没tilt）

最后call updateParams

##### updateParams()

对private member

```c++
Geometry _geometry{};
```

进行修改，逐个电机，赋值：

```c++
_geometry.rotors[i].position;
_geometry.rotors[i].axis;
```

关于轴，module.yaml 默认的就是 `AxisConfiguration::FixedUpwards`，即

```c++
		case AxisConfiguration::FixedUpwards:
			axis = Vector3f(0.f, 0.f, -1.f);
			break;
		}
```

以及CT与KM：

```c++
param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);
param_get(_param_handles[i].moment_ratio, &_geometry.rotors[i].moment_ratio);
```

现在 `_geometry` 拥有读取到的这些无人机参数。

##### addActuators(Configuration &configuration)

ControlAllocator::update_effectiveness_matrix_if_needed 定义的config就将在此处被修改。

```c++
bool ActuatorEffectivenessRotors::addActuators(Configuration &configuration) {
	int num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],
			    configuration.num_actuators_matrix[configuration.selected_matrix]);
	configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);
	return true;
}
```

接下来进入函数 `computeEffectivenessMatrix`

##### computeEffectivenessMatrix

首先回顾一下 configuration/config 的结构：

```c++
	struct Configuration {
		/**
		 * Add an actuator to the selected matrix, returning the index, or -1 on error
		 */
		int addActuator(ActuatorType type, const matrix::Vector3f &torque, const matrix::Vector3f &thrust);

		/**
		 * Call this after manually adding N actuators to the selected matrix
		 */
		void actuatorsAdded(ActuatorType type, int count);

		int totalNumActuators() const;

		/// Configured effectiveness matrix. Actuators are expected to be filled in order, motors first, then servos
		EffectivenessMatrix effectiveness_matrices[MAX_NUM_MATRICES];

		int num_actuators_matrix[MAX_NUM_MATRICES]; ///< current amount, and next actuator index to fill in to effectiveness_matrices
		ActuatorVector trim[MAX_NUM_MATRICES];

		ActuatorVector linearization_point[MAX_NUM_MATRICES];

		int selected_matrix;

		uint8_t matrix_selection_indexes[NUM_ACTUATORS * MAX_NUM_MATRICES];
		int num_actuators[(int)ActuatorType::COUNT];
	};
```

其中参数 `selected_matrix` 并没有被初始化。并且参考 1.1.1 中的函数 update_effectiveness_matrix_if_needed()，config的此成员确实没有被修改过，所以应为0.

所以 addActuators 函数的两个参数为：

```c++
configuration.effectiveness_matrices[0]
configuration.num_actuators_matrix[0]
```

* 对于多旋翼也合理，因为只有一个控制分配矩阵。
* 另一个 num_actuators_matrix 定义是`int num_actuators_matrix[MAX_NUM_MATRICES]`，其中 `static constexpr int MAX_NUM_MATRICES = 2;`，所以是一个二维数组。并且也没有被赋值，所以是两个0；

接下来是计算控制效率矩阵的函数（有删减）：

```c++
	int num_actuators = 0;
	for (int i = 0; i < math::min(NUM_ROTORS_MAX, geometry.num_rotors); i++) {
		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		++num_actuators;

		// Get rotor axis
		Vector3f axis = geometry.rotors[i].axis;
		// Normalize axis，略

		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		// Get coefficients
		float ct = geometry.rotors[i].thrust_coef;
		float km = geometry.rotors[i].moment_ratio;

		// Compute thrust generated by this rotor
		matrix::Vector3f thrust = ct * axis;

		// Compute moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
            // actuator_start_index = 0，见上文分析
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}
	}

	return num_actuators;  // 几个电机就是几，只要不超过8 NUM_ROTORS_MAX
```

对于四旋翼，4*3的循环。

所以最后的矩阵是类似下面这种的：

|          | Rotor1 | Rotor2 | Rotor3 | Rotor4 | Rotor x |
| -------- | ------ | ------ | ------ | ------ | ------- |
| Moment x | -1.52  | 1.39   | 1.52   | -1.39  | 0       |
| Moment y | 0.90   | -0.90  | 0.90   | -0.90  | 0       |
| Moment z | 0.83   | 0.83   | -0.83  | -0.83  | 0       |
| Thrust x | 0      | 0      | 0      | 0      | 0       |
| Thrust y | 0      | 0      | 0      | 0      | 0       |
| Thrust z | -6.9   | -6.9   | -6.9   | -6.9   | 0       |

是一个 16 * 6 的矩阵。

#### 1.3.3 derived class [`AcutatorEffectivenessMultirotor`](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessMultirotor.cpp)

注意到构造函数包含了一个初始化列表，和一个protected member `_mc_rotors`

```c++
class ActuatorEffectivenessMultirotor : public ModuleParams, public ActuatorEffectiveness
public:
	ActuatorEffectivenessMultirotor(ModuleParams *parent);
protected:
	ActuatorEffectivenessRotors _mc_rotors;

ActuatorEffectivenessMultirotor::ActuatorEffectivenessMultirotor(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this) {}
```

结合 [ControlActuator.cpp Line 217](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ControlAllocator.cpp#L217) 中的：

```c++
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;
```

说明构造AEM对象时，将ControlActuator对象作为参数传到了 parent 参数，定义在 ModuleParams 基类中（暂时不知道干什么用的）。暂时感觉就是单纯给AEM的类成员变量parent赋值为ControlActuator的指针，也符合对parent这个参数名如其意的定义。

之后使用当前 this指针，即AEM地址，实例化了一个  `ActuatorEffectivenessRotors`对象，对应AER构造函数的第一个参数，因为AEM也是 ModuleParams 的子类。

所以在构造AER对象的过程中，应该已经从机型的 

##### getEffectivenessMatrix(config)

external_update 参数，在 ControllAllocator 中称为 reason，对于多旋翼只有0和1两个值。

现在要对引用config进行修改。

对于多旋翼，see [AcutatorEffectivenessMultirotor.cpp Line 45](https://github.com/PX4/PX4-Autopilot/blob/v1.13.0/src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessMultirotor.cpp#L45)

```c++
bool ActuatorEffectivenessMultirotor::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update) {
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}
	// 所以多旋翼只在参数更新阶段调用了以下函数，addActuators
	// Motors
	const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

	return rotors_added_successfully;
}
```

所以实际对`config`的修改发生在 AER 类中的 setActuators 函数中。

## mixer (fall back)
