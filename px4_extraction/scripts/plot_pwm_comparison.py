#!/usr/bin/env python3
"""
Plot PX4-logged PWM vs extraction-computed PWM for each motor.

Reads the L3 trace CSV, applies the same thrust-curve-inverse + PWM mapping
that ActuatorOutputWrapper.cpp uses, and overlays both signals per motor.

Usage:
    python plot_pwm_comparison.py [trace.csv] [output.png]

Defaults:
    trace.csv  = ../test/data/l3_trace.csv  (relative to this script)
    output.png = ../doc/l3_pwm_comparison.png
"""

import sys
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # non-interactive backend for headless servers
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Thrust curve inverse — mirrors ThrustCurve.hpp applyThrustCurveInverse()
# ---------------------------------------------------------------------------
def thrust_curve_inverse(control: np.ndarray, thrust_factor: float) -> np.ndarray:
    """
    Apply thrust model factor inverse.

    Forward model: rel_thrust = f * x^2 + (1-f) * x
    Inverse (solving for x): x = -(1-f)/(2f) + sqrt((1-f)^2/(4f^2) + control/f)

    When f ≈ 0, the model is linear (identity).
    """
    x = control.copy()

    if thrust_factor > 1e-7:
        a = thrust_factor
        b = 1.0 - thrust_factor
        tmp1 = b / (2.0 * a)
        tmp2 = b * b / (4.0 * a * a)

        valid = np.isfinite(x) & (x > 1e-7)
        x[valid] = -tmp1 + np.sqrt(tmp2 + x[valid] / a)

    # Clamp to [0, 1]
    x = np.clip(x, 0.0, 1.0)
    return x


# ---------------------------------------------------------------------------
# PWM mapping — mirrors ActuatorOutputWrapper::mapPWM()
# ---------------------------------------------------------------------------
def map_pwm(motors: np.ndarray, pwm_min: float = 1000.0, pwm_max: float = 2000.0) -> np.ndarray:
    """
    Map normalized motor commands [0,1] → PWM [pwm_min, pwm_max].

    Steps (matching C++):
        1. Remap [0,1] → [-1,1]:  value = motor * 2 - 1
        2. Interpolate [-1,1] → [pwm_min, pwm_max]
        3. Round to integer
    """
    value = motors * 2.0 - 1.0  # [0,1] → [-1,1]
    pwm_f = pwm_min + (value + 1.0) / 2.0 * (pwm_max - pwm_min)
    return np.round(pwm_f).astype(float)


# ---------------------------------------------------------------------------
# Load CSV
# ---------------------------------------------------------------------------
def load_trace(csv_path: str):
    """Return dict of numpy arrays keyed by column name."""
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        columns = {col: [] for col in reader.fieldnames}
        for row in reader:
            for col in reader.fieldnames:
                columns[col].append(float(row[col]))
    return {k: np.array(v) for k, v in columns.items()}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_csv = os.path.join(script_dir, "..", "test", "data", "l3_trace.csv")
    default_out = os.path.join(script_dir, "..", "doc", "l3_pwm_comparison.png")

    csv_path = sys.argv[1] if len(sys.argv) > 1 else default_csv
    out_path = sys.argv[2] if len(sys.argv) > 2 else default_out

    print(f"Loading trace: {csv_path}")
    data = load_trace(csv_path)

    # Time axis in seconds (relative to first sample)
    t_us = data["timestamp_us"]
    t_s = (t_us - t_us[0]) / 1e6

    # --- Iris SITL defaults ---
    THRUST_MODEL_FACTOR = 0.0  # linear (Iris default)
    PWM_MIN = 1000.0
    PWM_MAX = 2000.0
    PWM_DISARMED = 900.0

    # Compute extraction PWM from logged normalized motors (isolated Stage 4)
    extraction_pwm = {}
    for m in range(4):
        motors = data[f"motor{m}"].copy()
        motors = thrust_curve_inverse(motors, THRUST_MODEL_FACTOR)
        pwm = map_pwm(motors, PWM_MIN, PWM_MAX)
        # Handle disarmed: where motor ≈ 0, PX4 outputs PWM_DISARMED
        # The C++ path always maps via the formula (armed=true), so we replicate that.
        extraction_pwm[m] = pwm

    # PX4 logged PWM
    px4_pwm = {m: data[f"pwm{m}"] for m in range(4)}

    # --- Detect actual PX4 PWM change-points (show publish rate) ---
    change_mask = {}
    for m in range(4):
        diff = np.diff(px4_pwm[m], prepend=px4_pwm[m][0] - 1)
        change_mask[m] = diff != 0.0

    n_samples = len(t_s)
    n_changes = sum(change_mask[0])
    print(f"Samples: {n_samples}, PX4 PWM change-points (motor 0): {n_changes}")

    # ---------- Plot ----------
    fig, axes = plt.subplots(5, 1, figsize=(14, 14), sharex=True,
                             gridspec_kw={"height_ratios": [1, 1, 1, 1, 0.8]})

    motor_colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

    for m in range(4):
        ax = axes[m]
        # PX4 logged PWM (staircase from slow publish rate)
        ax.step(t_s, px4_pwm[m], where="post", color="#1f77b4",
                linewidth=0.8, alpha=0.85, label="PX4 logged PWM")
        # Mark actual PX4 publish events
        ax.scatter(t_s[change_mask[m]], px4_pwm[m][change_mask[m]],
                   s=3, color="#1f77b4", alpha=0.4, zorder=3)

        # Extraction-computed PWM
        ax.plot(t_s, extraction_pwm[m], color="#ff7f0e",
                linewidth=0.6, alpha=0.85, label="Extraction PWM")

        ax.set_ylabel(f"Motor {m}\nPWM [µs]", fontsize=9)
        ax.set_ylim(PWM_DISARMED - 50, PWM_MAX + 50)
        ax.legend(loc="upper right", fontsize=7, framealpha=0.7)
        ax.grid(True, alpha=0.3)

    # --- Error subplot ---
    ax_err = axes[4]
    for m in range(4):
        err = extraction_pwm[m] - px4_pwm[m]
        ax_err.plot(t_s, err, linewidth=0.5, alpha=0.7, color=motor_colors[m],
                    label=f"Motor {m}")
    ax_err.axhline(0, color="black", linewidth=0.5, alpha=0.5)
    ax_err.axhline(100, color="red", linewidth=0.5, linestyle="--", alpha=0.4, label="±100 µs tol")
    ax_err.axhline(-100, color="red", linewidth=0.5, linestyle="--", alpha=0.4)
    ax_err.set_ylabel("PWM Error\n[µs]", fontsize=9)
    ax_err.set_xlabel("Time [s]", fontsize=10)
    ax_err.legend(loc="upper right", fontsize=7, framealpha=0.7, ncol=3)
    ax_err.grid(True, alpha=0.3)

    fig.suptitle(
        "L3 Validation: PX4 Logged PWM vs Extraction-Computed PWM\n"
        f"(Iris SITL, THR_MDL_FAC={THRUST_MODEL_FACTOR}, "
        f"PWM [{int(PWM_MIN)}–{int(PWM_MAX)}] µs, "
        f"{n_samples} samples)",
        fontsize=11, fontweight="bold"
    )

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"Saved plot to: {out_path}")


if __name__ == "__main__":
    main()
