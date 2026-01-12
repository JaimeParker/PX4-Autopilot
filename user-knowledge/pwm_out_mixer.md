# actuator sp to PWM

ControlAllocator发出来的actuator_sp到pwm的过程。阅读顺序，2-1-3

**PX4版本 release/1.13** （控制分配的分析按1.13.0算的，应该差不多）

## 1. mixer_module

mixer_module是shared_lib，这里只对函数做分析，流程直接看 **2. pwm_out**

关键函数是update，也是接受actuator sp与发送pwm值的关键，PWMOut只是做了个调用，和其他处理

#### update()

```c++
bool MixingOutput::update() {
	// _use_dynamic_mixing = _param_sys_ctrl_alloc.get();
	if (_use_dynamic_mixing) {
		return updateDynamicMixer();
	} else {
		return updateStaticMixer();
	}
}
```

根据 SYS_CTRL_ALLOC 值决定。

#### updateDynamicMixer()

有很多类似 safety check 的东西，比较重要的应该是：

```c++
	// update topics
	for (int i = 0; i < MAX_ACTUATORS && _function_allocated[i]; ++i) {
		_function_allocated[i]->update();
	}	

	// get output values
	float outputs[MAX_ACTUATORS];
	bool all_disabled = true;
	_reversible_mask = 0;

	for (int i = 0; i < _max_num_outputs; ++i) {
		if (_functions[i]) {
			all_disabled = false;

			if (_armed.armed || (_armed.prearmed && _functions[i]->allowPrearmControl())) {
				outputs[i] = _functions[i]->value(_function_assignment[i]);

			} else {
				outputs[i] = NAN;
			}

			_reversible_mask |= (uint32_t)_functions[i]->reversible(_function_assignment[i]) << i;

		} else {
			outputs[i] = NAN;
		}
	}
```

应该把 actuator_sp 的值拿到了 output 中，为什么这么说呢，因为已知 ControlAllocator 发出来的是：

```c++
_actuator_motors_pub.publish(actuator_motors);
uORB::Publication<actuator_motors_s>	_actuator_motors_pub{ORB_ID(actuator_motors)};
```

所以在 PX4 root 下搜索 `ORB_ID(actuator_motors)`，有pub必须得有sub

在Release1.13版本中出现了4处：

* actuator_test.hpp
* **functions.cpp**
* mixer_modules_test.cpp
* ControlAllocator.hpp

一看就是用来PWM的function，比如 `10017_iris_ctrlalloc` 中的：

```
param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104
```

而且funciton的构造里也写明了：

```c++
FunctionMotors::FunctionMotors(const Context &context)
	: _topic(&context.work_item, ORB_ID(actuator_motors)),
	  _thrust_factor(context.thrust_factor)
{
	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		_data.control[i] = NAN;
	}
}
```

继续updateDynamicMixer函数，

```c++
	if (!all_disabled) {
		if (!_armed.armed && !_armed.manual_lockdown) {
			_actuator_test.overrideValues(outputs, _max_num_outputs);
		}

		limitAndUpdateOutputs(outputs, has_updates);
	}

	return true;
```

所以在limitAndUpdateOutputs及其调用的**setAndPublishActuatorOutputs**函数中完成了处理及发布。



---

（可以先跳过这部分）回过头来，

已知对于多旋翼，actuator_motors 出来是$[0, 1]$ 的，但是 function 在此做了处理，详见 3. update 和 updateValues

同时关注到头文件里定义的：

```c++
	/* SYS_CTRL_ALLOC == 1 */
	FunctionProviderBase *_function_allocated[MAX_ACTUATORS] {}; ///< unique allocated functions
	FunctionProviderBase *_functions[MAX_ACTUATORS] {}; ///< currently assigned functions
	OutputFunction _function_assignment[MAX_ACTUATORS] {};
```

在updateDynamicMixer()函数中，负责执行了FunctionMotors类的update的应该是 `_function_allocated`，这名字也感觉像是 ControlAllocator之后的，但是用于赋值的却是：

```c++
outputs[i] = _functions[i]->value(_function_assignment[i]);
```

如果_functions中的还是原始的 $[0, 1]$，那么 `THR_MDL_FAC` 参数将没派上用场，且最后变成 PWM 值也对不上，所以只能是已经变成 $[-1, 1]$ 的，只不过没找到在哪变的，这是个疑点，



#### limitAndUpdateOutputs()

```c++
	 else {
		// the output limit call takes care of out of band errors, NaN and constrains
		output_limit_calc(_throttle_armed || _actuator_test.inTestMode(), _max_num_outputs, outputs);
	}

	/* now return the outputs to the driver */
	if (_interface.updateOutputs(stop_motors, _current_output_value, _max_num_outputs, has_updates)) {
		actuator_outputs_s actuator_outputs{};
		setAndPublishActuatorOutputs(_max_num_outputs, actuator_outputs);

		updateLatencyPerfCounter(actuator_outputs);
	}
```

进入计算函数output_limit_calc，之后基本就认为return了

#### setAndPublishActuatorOutputs()

先说发布函数，因为比较简单

```c++
void MixingOutput::setAndPublishActuatorOutputs(unsigned num_outputs, actuator_outputs_s &actuator_outputs) {
	actuator_outputs.noutputs = num_outputs;

	for (size_t i = 0; i < num_outputs; ++i) {
		actuator_outputs.output[i] = _current_output_value[i];
	}

	actuator_outputs.timestamp = hrt_absolute_time();
	// uORB::PublicationMulti<actuator_outputs_s> _outputs_pub{ORB_ID(actuator_outputs)};
	_outputs_pub.publish(actuator_outputs);
}
```

直接对 **_current_output_value** 进行发布，而对此类成员赋值就在函数 output_limit_calc

#### output_limit_calc()

状态机来了。

根据定义，大部分时间应该处于 `ON` 状态，可以参考[pwm_limit状态机 gitbook](https://shnuzxd.gitbooks.io/px4-development-guide/content/zh/concept/pwm_limit.html)

```c++
	case OutputLimitState::ON:
		for (int i = 0; i < num_channels; i++) {
			_current_output_value[i] = output_limit_calc_single(i, output[i]);
		}
```

之后针对每个actuator sp，也就是这里的output，调用output_limit_calc_single函数

#### output_limit_calc_single()

```c++
uint16_t MixingOutput::output_limit_calc_single(int i, float value) const
{
	// check for invalid / disabled channels
	if (!PX4_ISFINITE(value)) {
		return _disarmed_value[i];
	}

	if (_reverse_output_mask & (1 << i)) {
		value = -1.f * value;
	}

	uint16_t effective_output = value * (_max_value[i] - _min_value[i]) / 2 + (_max_value[i] + _min_value[i]) / 2;

	// last line of defense against invalid inputs
	return math::constrain(effective_output, _min_value[i], _max_value[i]);
}
```

基本上就是最后两行，不反转就是直接拿过来用。

以`_max_value`为例，看设置的是哪个PWM值。

```c++
void MixingOutput::setAllMaxValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].max = PARAM_INVALID;
		_max_value[i] = value;
	}
}
```

接下来找哪里调用了此函数：

![](images/setAllMaxValues.png)

本来以为会是PWMOut，结果

```c++
	if (!_mixing_output.useDynamicMixing()) {
		_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
		_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);
	}
```

那就只能在自己源代码里某处被赋值了，最后找到了函数 updateParams()

#### updateParams()

```c++
void MixingOutput::updateParams()
{
	ModuleParams::updateParams();

	// update mixer if we have one
	if (_mixers) {
		if (_param_mot_slew_max.get() <= FLT_EPSILON) {
			_mixers->set_max_delta_out_once(0.f);
		}

		_mixers->set_thrust_factor(_param_thr_mdl_fac.get());
		_mixers->set_airmode((Mixer::Airmode)_param_mc_airmode.get());
	}

	if (_use_dynamic_mixing) {
        // ...
			if (_param_handles[i].min != PARAM_INVALID && param_get(_param_handles[i].min, &val) == 0) {
				_min_value[i] = val;
			}

			if (_param_handles[i].max != PARAM_INVALID && param_get(_param_handles[i].max, &val) == 0) {
				_max_value[i] = val;
			}

			if (_min_value[i] > _max_value[i]) {
				uint16_t tmp = _min_value[i];
				_min_value[i] = _max_value[i];
				_max_value[i] = tmp;
			}
        // ...
	}
}
```

首先掉了个基类的updateParams更新了全部参数。

关于参数 `_mixers`，根据对整个类判断，以及`iris_ctrlalloc`的如下设置：

```
set MIXER skip
set MIXER_AUX none
```

和对updateStaticMixer()方法的对比，我觉得开了 SYS_CTRL_ALLOC 之后，应该是个空指针。

所以**直接进入下面的if condition**。

关于`param_get(_param_handles[i].max, &val)`取到的值，见MixingOutput()与initParamHandles()

#### MixingOutput()

```c++
	_use_dynamic_mixing = _param_sys_ctrl_alloc.get();

	if (_use_dynamic_mixing) {
		initParamHandles();

		for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
			_failsafe_value[i] = UINT16_MAX;
		}

		updateParams();
		_outputs_pub.advertise();

	} 
```

关注函数initParamHandles()；

#### initParamHandles()

```c++
void MixingOutput::initParamHandles()
{
	char param_name[17];

	for (unsigned i = 0; i < _max_num_outputs; ++i) {
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "FUNC", i + 1);
		_param_handles[i].function = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "DIS", i + 1);
		_param_handles[i].disarmed = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "MIN", i + 1);
		_param_handles[i].min = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "MAX", i + 1);
		_param_handles[i].max = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "FAIL", i + 1);
		_param_handles[i].failsafe = param_find(param_name);
	}

	snprintf(param_name, sizeof(param_name), "%s_%s", _param_prefix, "REV");
	_param_handle_rev_range = param_find(param_name);
}
```

对比 `ROMFS/px4fmu_common/init.d/rc.mc_defaults` 中的

```
param set-default PWM_MAIN_MAX 1950
param set-default PWM_MAIN_MIN 1075
```

所以这一段作用应该是对0到_max_num_outputs做循环，然后

* 给`param_name`赋值，大概赋值成 `PWM_MAIN_MAX0`这样的string
* 之后根据这个string给`_param_handles[i].max`赋值

这时候再看一下 `iris_ctrlalloc`：

```
. ${R}etc/init.d/rc.mc_defaults

param set-default SYS_CTRL_ALLOC 1


param set-default CA_AIRFRAME 0

param set-default CA_ROTOR_COUNT 4
param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR0_KM 0.12
param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.20
param set-default CA_ROTOR1_KM 0.12
param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR2_KM -0.12
param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.20
param set-default CA_ROTOR3_KM -0.12

param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104

set MIXER skip
set MIXER_AUX none
```

所以无论是 `rc.mc_defaults` 还是 `10017_iris_ctrlalloc`，都没有定义 `PWM_MAIN_MAX0`

这种情况下，可能使用的就是 [src/drivers/drv_pwm_output.h](https://github.com/PX4/PX4-Autopilot/blob/release/1.13/src/drivers/drv_pwm_output.h#L107) 中定义的2000；

```c++
/**
 * Default maximum PWM in us
 */
#define PWM_DEFAULT_MAX 2000
```

## 2. [pwm_out](https://github.com/PX4/PX4-Autopilot/tree/release/1.13/src/drivers/pwm_out)

还是围绕着主函数Run展开，看来这是PX4的惯用方法。

#### Run()

```c++
	// _use_dynamic_mixing = _param_sys_ctrl_alloc.get();
	// 所以此判断完全看SYS_CTRL_ALLOC的值
	if (!_mixing_output.useDynamicMixing()) {
		// push backup schedule
		ScheduleDelayed(_backup_schedule_interval_us);
	}
```

之后直接就到了

```c++
_mixing_output.update();
```

这里面涉及的东西就太多了。

剩下的部分，根据注释可以看出，一般不会触发，所以关键就在这个update里。

比较有意思的是：

```c++
		// update parameters from storage
		if (_mixing_output.useDynamicMixing()) { // do not update PWM params for now (was interfering with VTOL PWM settings)
			update_params();
		}
```

结合下面的update_params()，当SYS_CTRL_ALLOC=1时，应该调用的是基类的更新函数。

#### init()

没见在哪调用了

#### update_params()

```c++
void PWMOut::update_params() {
	updateParams();

	// 所以 SYS_CTRL_ALLOC=1 时，不会执行下面的代码，那该怎么更新参数呢？
	if (_mixing_output.useDynamicMixing()) {
		return;
	}
```

这里面调用的 update_params 是基类方法，作用是给所有派生类都执行参数更新，

```c++
	/**
	 * @brief Call this method whenever the module gets a parameter change notification.
	 *        It will automatically call updateParams() for all children, which then call updateParamsImpl().
	 */
	virtual void updateParams()
	{
		for (const auto &child : _children) {
			child->updateParams();
		}

		updateParamsImpl();
	}
```

可以看一下如果SYS_CTRL_ALLOC = 0，后续的过程：

```c++
if (_class_instance == CLASS_DEVICE_PRIMARY) {
		prefix = "PWM_MAIN";

		param_get(param_find("PWM_MAIN_MIN"), &pwm_min_default);
		param_get(param_find("PWM_MAIN_MAX"), &pwm_max_default);
		param_get(param_find("PWM_MAIN_DISARM"), &pwm_disarmed_default);
		param_get(param_find("PWM_MAIN_RATE"), &pwm_rate_default);
		param_get(param_find("PWM_MAIN_OUT"), &pwm_default_channels);
	}
```

看起来很正常，随后也会给加载到的pwm min和max做一个限幅，不能太小不能太大。

## 3. functinos

#### FunctionMotors()

```c++
FunctionMotors::FunctionMotors(const Context &context)
	: _topic(&context.work_item, ORB_ID(actuator_motors)),
	  _thrust_factor(context.thrust_factor)
{
	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		_data.control[i] = NAN;
	}
}
```

订阅了 ControlAllocator 发出的消息，注意范围是 $[0, 1]$ 的

#### update()

```c++
void FunctionMotors::update()
{
	if (_topic.update(&_data)) {
		updateValues(_data.reversible_flags, _thrust_factor, _data.control, actuator_motors_s::NUM_CONTROLS);
	}
}
```

此函数的关键，updateValues

#### updateValues()

```c++
void FunctionMotors::updateValues(uint32_t reversible, float thrust_factor, float *values, int num_values)
{
	if (thrust_factor > 0.f && thrust_factor <= 1.f) {
		// thrust factor
		//  rel_thrust = factor * x^2 + (1-factor) * x,
		const float a = thrust_factor;
		const float b = (1.f - thrust_factor);

		// don't recompute for all values (ax^2+bx+c=0)
		const float tmp1 = b / (2.f * a);
		const float tmp2 = b * b / (4.f * a * a);

		for (int i = 0; i < num_values; ++i) {
			float control = values[i];

			if (control > 0.f) {
				values[i] = -tmp1 + sqrtf(tmp2 + (control / a));

			} else if (control < -0.f) {
				values[i] =  tmp1 - sqrtf(tmp2 - (control / a));

			} else {
				values[i] = 0.f;
			}
		}
	}

	for (int i = 0; i < num_values; ++i) {
		if ((reversible & (1u << i)) == 0) {
			if (values[i] < -FLT_EPSILON) {
				values[i] = NAN;

			} else {
				// remap from [0, 1] to [-1, 1]
				values[i] = values[i] * 2.f - 1.f;
			}
		}
	}
}
```

这里面包括了推力参数 `THR_MDL_FAC` 的使用，和将 actuator_motors 这个 $[0,1 ]$ 的量变为 $[-1, 1]$ 的过程。
