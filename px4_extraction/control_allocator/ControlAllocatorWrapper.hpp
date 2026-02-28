/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Extraction Project. All rights reserved.
 *   BSD 3-Clause License (same as PX4)
 *
 ****************************************************************************/

/**
 * @file ControlAllocatorWrapper.hpp
 *
 * Clean C++ API for the PX4 control allocator.
 * Wraps ControlAllocationSequentialDesaturation with the project's data-driven pattern.
 */

#pragma once

#include "ControlAllocatorTypes.hpp"
#include "ControlAllocationSequentialDesaturation.hpp"

namespace px4_extraction {

class ControlAllocatorWrapper {
public:
	ControlAllocatorWrapper();
	~ControlAllocatorWrapper() = default;

	/**
	 * Configure the allocator (geometry, airmode, bounds, etc.).
	 * Rebuilds the effectiveness matrix from geometry.
	 */
	void setParams(const ControlAllocatorParams &params);

	/**
	 * Get current parameters.
	 */
	ControlAllocatorParams getParams() const { return _params; }

	/**
	 * Run one allocation step.
	 * Maps torque_sp + thrust_sp → motor commands [0,1] + saturation flags.
	 *
	 * @param input  Torque/thrust setpoints + dt
	 * @param output Motor commands + saturation feedback
	 */
	void update(const ControlAllocatorInput &input, ControlAllocatorOutput &output);

	/**
	 * Get the effectiveness matrix (6×16, only first num_motors columns populated).
	 */
	const ActuatorEffectiveness::EffectivenessMatrix &getEffectivenessMatrix() const
	{ return _allocation.getEffectivenessMatrix(); }

	/**
	 * Get the mix matrix (16×6 pseudo-inverse, normalized).
	 * Useful for debugging/inspection.
	 */
	const matrix::Vector<float, 16> &getActuatorSetpoint() const
	{ return _allocation.getActuatorSetpoint(); }

private:
	void applyParams();

	ControlAllocationSequentialDesaturation _allocation;
	ControlAllocatorParams _params;
	int _num_motors{4};
};

} // namespace px4_extraction
