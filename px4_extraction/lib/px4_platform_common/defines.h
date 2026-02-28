/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Minimal replacement for px4_platform_common/defines.h
 * Contains only the defines needed by matrix library and mathlib
 *
 ****************************************************************************/

#pragma once

#include <cmath>

// High precision Pi constant
#define M_PI_PRECISE        3.141592653589793238462643383279502884

// Finite check macros
static inline constexpr bool PX4_ISFINITE(float x) { return std::isfinite(x); }
static inline constexpr bool PX4_ISFINITE(double x) { return std::isfinite(x); }
