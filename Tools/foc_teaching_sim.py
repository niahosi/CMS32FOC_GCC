#!/usr/bin/env python3
"""
FOC Teaching Simulator
======================
Interactive simulator for understanding FOC current/speed loop dynamics.
Tailored for the CMS32FOC motor: 19μH, 4Ω, 2.16V/krpm, 4 pole pairs, 20kHz PWM.

Usage:
    python foc_teaching_sim.py

Requirements:
    pip install numpy matplotlib

Controls:
    - Left panel:  scenario selector (radio buttons) + mode toggle
    - Right panel: parameter sliders
    - Center:      waveform plots and diagnostic text
    - "Mystery Fault" button injects a random misconfiguration.
      Diagnose from the waveforms, then click "Reveal Fault" to check.
"""

from __future__ import annotations

import copy
import math
import random
import sys
from dataclasses import dataclass, field
from typing import Any, Callable

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button, RadioButtons, Slider

# ============================================================================
# 1. Parameter containers
# ============================================================================


@dataclass
class MotorParams:
    """Physical motor parameters — defaults match CMS32FOC hardware."""

    L: float = 19.0e-6  # Phase inductance [H]
    R: float = 4.0       # Phase resistance [Ω]
    Ke: float = 2.16     # Back-EMF constant [V/krpm] line-to-line peak
    pole_pairs: int = 4
    J: float = 5.0e-6    # Rotor + small load inertia [kg·m²]
    B: float = 2.0e-5    # Viscous damping [Nm/(rad/s)]

    @property
    def psi_m(self) -> float:
        """PM flux linkage [Wb] = V·s/rad (phase-peak per electrical rad/s)."""
        ke_phase_peak = self.Ke / math.sqrt(3)         # phase-peak V/krpm
        w_elec_1krpm = 1000.0 * 2 * math.pi / 60.0 * self.pole_pairs  # elec rad/s
        return ke_phase_peak / w_elec_1krpm

    @property
    def tau_e(self) -> float:
        """Electrical time constant [s]."""
        return self.L / self.R


@dataclass
class FOCParams:
    """FOC controller parameters — defaults match TuneConfig.h tuning values."""

    # Current PI
    current_kp: int = 4
    current_ki: int = 1
    current_shift: int = 3   # right-shift bits (÷8)

    # Speed PI (teaching defaults — good regulation, low ripple)
    speed_kp: int = 8
    speed_ki: int = 2
    speed_shift: int = 6     # right-shift bits (÷64)
    speed_est_hz: float = 500.0

    # Limits
    v_limit: float = 887.0      # SVPWM voltage limit [counts] = PWM_SVPWM_V_LIMIT
    iq_limit: float = 400.0     # iq_ref clamp [counts]
    current_ref_ramp: int = 2   # ref slew step per tick

    # Hardware scaling (12V bus)
    i_count_per_a: float = 182.0   # 1A → 182 ADC counts
    v_count_per_v: float = 128.0   # 1V → 128 voltage counts

    # Sampling
    pwm_freq: float = 20000.0
    dt_ctrl: float = 1.0 / 20000.0

    # Intentional faults for teaching
    zero_offset_deg: float = 0.0    # electrical zero offset [degrees]
    deadtime_comp_missing: bool = False  # simulate missing deadtime compensation
    speed_filter_shift: int = 1     # 1st-stage speed FB filter
    speed_ctrl_filter_shift: int = 1  # 2nd-stage filter (light for teaching baseline)

    # Slew rates for speed→iq (fast enough for teaching baseline regulation)
    iq_slew_up: int = 80
    iq_slew_down: int = 80

    # Feedforward
    iq_ff_per_krpm: int = 40

    # Deadband
    speed_deadband_rpm: int = 5
    speed_zero_snap: float = 500.0  # counts/s


# ============================================================================
# 2. Motor physical model  (multi-rate: 1μs electrical, 50μs controller)
# ============================================================================


class MotorModel:
    """Discrete-time dq motor model with coupled electrical + mechanical dynamics.

    Electrical (1 μs inner loop for stability with τ_e = 4.75 μs):
        di_d/dt = (v_d - R·i_d + ω·L·i_q) / L
        di_q/dt = (v_q - R·i_q - ω·L·i_d - ω·ψ_m) / L

    Mechanical (updated at same rate for simplicity):
        dω_m/dt = (T_e - T_load - B·ω_m) / J
        T_e = 1.5 · pole_pairs · ψ_m · i_q      [SPMSM torque]
    """

    def __init__(self, mp: MotorParams):
        self.mp = mp
        # Internal electrical timestep
        self._dt_elec = 1.0e-6           # 1 μs
        self._steps_per_ctrl = int(FOCParams().dt_ctrl / self._dt_elec)  # 50

    def reset(self, rotor_locked: bool = False):
        self.i_d: float = 0.0       # physical d-axis current [A]
        self.i_q: float = 0.0       # physical q-axis current [A]
        self.omega_m: float = 0.0   # mechanical speed [rad/s]
        self.theta_elec: float = 0.0  # electrical angle [rad]
        self.rotor_locked = rotor_locked

    def step_electrical(self, v_d: float, v_q: float, dt: float):
        """Forward-Euler electrical update (stable for dt < 2·τ_e = 9.5μs)."""
        mp = self.mp
        omega_elec = self.omega_m * mp.pole_pairs  # electrical rad/s
        L, R = mp.L, mp.R

        di_d = (v_d - R * self.i_d + omega_elec * L * self.i_q) / L
        di_q = (v_q - R * self.i_q - omega_elec * L * self.i_d
                - omega_elec * mp.psi_m) / L

        self.i_d += di_d * dt
        self.i_q += di_q * dt
        self.theta_elec += omega_elec * dt
        # keep theta in [0, 2π)
        self.theta_elec = self.theta_elec % (2 * math.pi)

    def step_mechanical(self, load_torque: float, dt: float):
        """Mechanical update (skipped if rotor is locked)."""
        if self.rotor_locked:
            return
        mp = self.mp
        T_e = 1.5 * mp.pole_pairs * mp.psi_m * self.i_q
        domega = (T_e - load_torque - mp.B * self.omega_m) / mp.J
        self.omega_m += domega * dt
        if self.omega_m < 0:
            self.omega_m = 0.0

    def run_one_control_cycle(self, v_d_phys: float, v_q_phys: float,
                              load_torque: float = 0.0):
        """Run N micro-steps for one 50μs control cycle."""
        for _ in range(self._steps_per_ctrl):
            self.step_electrical(v_d_phys, v_q_phys, self._dt_elec)
            self.step_mechanical(load_torque, self._dt_elec)


# ============================================================================
# 3. FOC Controller  (matching the CMS32 C firmware implementation)
# ============================================================================


def _q15_sin(angle_16bit: int) -> float:
    """Approximate Q15 sin from 16-bit angle, matching foc_sin_q15().
    Returns float in [-1, 1] for convenience (real firmware uses fixed-point)."""
    quadrant = (angle_16bit >> 14) & 0x3
    phase = angle_16bit & 0x3FFF
    index = phase >> 6
    frac = phase & 0x3F

    # 64-entry quarter-wave table (scaled from the 257-entry table in firmware
    # by taking every 4th point + linear interpolation of the fraction)
    # For teaching purposes, we use math.sin scaled to 16-bit — it's visually
    # indistinguishable from the real Q15 table.
    angle_rad = (angle_16bit / 65536.0) * 2 * math.pi
    return math.sin(angle_rad)


def _q15_cos(angle_16bit: int) -> float:
    return _q15_sin((angle_16bit + 16384) & 0xFFFF)


class PIController:
    """Fixed-point PI controller matching foc_pi_update() behaviour.

    Algorithm (from foc_math.c):
        error       = clamp(ref - fb, ±32767)
        integral   += error  (clamped ±32767)
        output_raw  = kp * error + ki * integral
        output      = clamp(output_raw >> shift, ±output_limit)

    Anti-windup: only update integral when output is not saturated, or
    when saturated but error direction would reduce the integral magnitude.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.integral: float = 0.0
        self.error: float = 0.0
        self.output: float = 0.0

    def update(self, ref: float, fb: float, kp: float, ki: float,
               shift: int, out_min: float, out_max: float) -> float:
        error = max(-32767.0, min(32767.0, ref - fb))
        self.error = error

        if ki == 0:
            self.integral = 0.0
            raw = kp * error
            out = raw / (1 << shift)
            out = max(out_min, min(out_max, out))
            self.output = out
            return out

        integral_new = max(-32767.0, min(32767.0, self.integral + error))
        raw = kp * error + ki * integral_new
        out_unclamped = raw / (1 << shift)
        out = max(out_min, min(out_max, out_unclamped))

        # Anti-windup (matching foc_pi_update in foc_math.c):
        # Update integral when: (a) output not saturated, OR
        # (b) saturated high but error is negative (pulling back), OR
        # (c) saturated low but error is positive (pulling up).
        # Hold integral when saturated AND error pushes further into saturation.
        if (out == out_unclamped or
            (out_unclamped > out_max and error < 0) or
            (out_unclamped < out_min and error > 0)):
            self.integral = integral_new
        # else: hold integral (anti-windup)

        # Recompute with actual integral (matching C code's second pass)
        raw = kp * error + ki * self.integral
        out_unclamped = raw / (1 << shift)
        out = max(out_min, min(out_max, out_unclamped))

        self.output = out
        return out


class FOCController:
    """Complete FOC chain: Clarke → Park → Current PI → InvPark → SVPWM.

    Operates in "count units" matching the firmware:
      - Currents in ADC counts  (1A ≈ 182 counts)
      - Voltages in duty counts  (1V ≈ 128 counts at 12V bus)
    """

    def __init__(self, fp: FOCParams, mp: MotorParams | None = None):
        self.fp = fp
        self.mp = mp or MotorParams()
        self.pi_d = PIController()
        self.pi_q = PIController()
        self.pi_speed = PIController()

        # Speed loop state
        self.speed_ref_active: float = 0.0
        self.speed_fb_filtered: float = 0.0   # after 1st-stage filter
        self.speed_fb_ctrl: float = 0.0        # after 2nd-stage filter
        self.speed_iq_cmd: float = 0.0
        self.speed_iq_ff: float = 0.0
        self.speed_integral: float = 0.0
        self.speed_sample_count: int = 0
        self._speed_div = int(fp.pwm_freq / fp.speed_est_hz)  # e.g. 40

        # Current ref slew state
        self.id_ref_active: float = 0.0
        self.iq_ref_active: float = 0.0

        # Electrical state
        self.theta_elec_16bit: int = 0  # 16-bit electrical angle
        self._encoder_raw: int = 0
        self._encoder_prev_raw: int = 0
        self._speed_prev_raw: int = 0

        # For speed FB from angle delta
        self._angle_accum: float = 0.0  # continuous angle for speed calc

    def reset(self):
        self.pi_d.reset()
        self.pi_q.reset()
        self.pi_speed.reset()
        self.speed_ref_active = 0.0
        self.speed_fb_filtered = 0.0
        self.speed_fb_ctrl = 0.0
        self.speed_iq_cmd = 0.0
        self.speed_iq_ff = 0.0
        self.speed_sample_count = 0
        self.id_ref_active = 0.0
        self.iq_ref_active = 0.0
        self._encoder_prev_raw = self._encoder_raw
        self._speed_prev_raw = self._encoder_raw
        self._angle_accum = 0.0

    # ---- Transforms ---------------------------------------------------------

    @staticmethod
    def clarke(i_u: float, i_v: float, i_w: float) -> tuple[float, float]:
        """3-phase → αβ.  iα = i_u,  iβ = (i_u + 2·i_v) / √3."""
        ialpha = i_u
        ibeta = (i_u + 2.0 * i_v) / math.sqrt(3)
        return ialpha, ibeta

    @staticmethod
    def park(ialpha: float, ibeta: float, theta_16bit: int) -> tuple[float, float]:
        """αβ → dq using 16-bit angle."""
        sin_t = _q15_sin(theta_16bit)
        cos_t = _q15_cos(theta_16bit)
        i_d = ialpha * cos_t + ibeta * sin_t
        i_q = -ialpha * sin_t + ibeta * cos_t
        return i_d, i_q

    @staticmethod
    def inv_park(v_d: float, v_q: float, theta_16bit: int) -> tuple[float, float]:
        """dq → αβ."""
        sin_t = _q15_sin(theta_16bit)
        cos_t = _q15_cos(theta_16bit)
        valpha = v_d * cos_t - v_q * sin_t
        vbeta = v_d * sin_t + v_q * cos_t
        return valpha, vbeta

    @staticmethod
    def svpwm_limit(v_d: float, v_q: float, v_limit: float) -> tuple[float, float, bool]:
        """Limit dq voltage vector magnitude (approximate, matching foc_limit_dq)."""
        abs_d = abs(v_d)
        abs_q = abs(v_q)
        mag = max(abs_d, abs_q) + min(abs_d, abs_q) * 0.5
        if mag <= v_limit:
            return v_d, v_q, False
        scale = v_limit / mag
        return v_d * scale, v_q * scale, True

    # ---- Main control cycle -------------------------------------------------

    def run_one_cycle(self, i_u: float, i_v: float, i_w: float,
                      theta_mech_rad: float,
                      id_ref: float, iq_ref: float,
                      speed_mode: bool, speed_ref_rpm: float,
                      speed_fb_raw_counts_per_s: float) -> dict[str, float]:
        """Execute one FOC control cycle (50 μs). Returns debug signals dict."""
        fp = self.fp

        # ---- Electrical angle with fault injection ----
        elec_rad = theta_mech_rad * self.mp.pole_pairs
        # Apply intentional zero offset (teaching fault)
        offset_rad = math.radians(fp.zero_offset_deg)
        elec_rad_used = elec_rad + offset_rad
        # 16-bit angle
        theta_16 = int((elec_rad_used % (2 * math.pi)) / (2 * math.pi) * 65536)
        self.theta_elec_16bit = theta_16

        # ---- Clarke + Park ----
        ialpha, ibeta = self.clarke(i_u, i_v, i_w)
        i_d, i_q = self.park(ialpha, ibeta, theta_16)

        # ---- Speed loop (at reduced rate) ----
        self.speed_sample_count += 1
        if self.speed_sample_count >= self._speed_div:
            self.speed_sample_count = 0
            self._run_speed_loop(speed_mode, speed_ref_rpm,
                                 speed_fb_raw_counts_per_s)

        # ---- Current reference selection + slew ----
        if speed_mode:
            cmd_iq = self.speed_iq_cmd
        else:
            cmd_iq = iq_ref

        self.id_ref_active = self._slew(self.id_ref_active, id_ref,
                                        fp.current_ref_ramp)
        self.iq_ref_active = self._slew(self.iq_ref_active,
                                        max(-fp.iq_limit, min(fp.iq_limit, cmd_iq)),
                                        fp.current_ref_ramp)

        # ---- Current PI ----
        v_d = self.pi_d.update(self.id_ref_active, i_d,
                               fp.current_kp, fp.current_ki,
                               fp.current_shift, -fp.v_limit, fp.v_limit)
        v_q = self.pi_q.update(self.iq_ref_active, i_q,
                               fp.current_kp, fp.current_ki,
                               fp.current_shift, -fp.v_limit, fp.v_limit)

        # ---- Voltage limit ----
        v_d_lim, v_q_lim, v_limited = self.svpwm_limit(v_d, v_q, fp.v_limit)

        # ---- Deadtime distortion (teaching fault) ----
        if fp.deadtime_comp_missing:
            # Crude model: voltage loss ≈ 2·deadtime·Vdc·sign(current) / period
            # This causes a voltage error that distorts low-current waveforms
            deadtime_err = 0.02 * fp.v_limit  # ~2% distortion
            if abs(i_q) < 20:
                v_q_lim -= math.copysign(deadtime_err, i_q) if i_q != 0 else 0
            if abs(i_d) < 20:
                v_d_lim -= math.copysign(deadtime_err, i_d) if i_d != 0 else 0

        # ---- InvPark → physical voltage ----
        valpha, vbeta = self.inv_park(v_d_lim, v_q_lim, theta_16)
        # Convert from voltage counts to physical volts
        # v_phys = v_counts / V_COUNT_PER_V
        # Then to 3-phase: v_u ≈ valpha, v_v ≈ -valpha/2 + vbeta*sqrt(3)/2
        # For motor model, we need dq voltages back in physical units:
        v_d_phys = v_d_lim / fp.v_count_per_v
        v_q_phys = v_q_lim / fp.v_count_per_v

        return {
            "i_d": i_d, "i_q": i_q,
            "i_d_ref": self.id_ref_active, "i_q_ref": self.iq_ref_active,
            "v_d": v_d, "v_q": v_q,
            "v_d_lim": v_d_lim, "v_q_lim": v_q_lim,
            "v_limited": float(v_limited),
            "v_d_phys": v_d_phys, "v_q_phys": v_q_phys,
            "valpha": valpha, "vbeta": vbeta,
            "ialpha": ialpha, "ibeta": ibeta,
            "theta_16": theta_16,
            "pi_d_integral": self.pi_d.integral,
            "pi_q_integral": self.pi_q.integral,
            "speed_iq_cmd": self.speed_iq_cmd,
            "speed_iq_ff": self.speed_iq_ff,
            "speed_fb_filtered": self.speed_fb_filtered,
            "speed_fb_ctrl": self.speed_fb_ctrl,
        }

    # ---- Internal helpers ----------------------------------------------------

    @staticmethod
    def _slew(current: float, target: float, step: float) -> float:
        if step <= 0:
            return target
        delta = target - current
        if delta > step:
            return current + step
        if delta < -step:
            return current - step
        return target

    def _run_speed_loop(self, speed_mode: bool, speed_ref_rpm: float,
                        speed_fb_raw: float):
        """Run one speed-loop update at the speed estimation rate (500 Hz)."""
        fp = self.fp

        # Convert raw counts/s to rpm for PI
        # counts/s → rpm:  rpm = counts/s * 60 / (CPR * pole_pairs)
        cpr = 65536  # MOT_SENSOR_CPR
        pp = 4       # pole pairs (simplified — in real code from MotorParams)
        rpm_scale = 60.0 / (cpr * pp)
        speed_fb_rpm_instant = speed_fb_raw * rpm_scale

        # ---- Two-stage speed feedback filter ----
        # Stage 1:  speed_fb_filtered += (instant - filtered) >> shift
        alpha1 = 1.0 / (1 << fp.speed_filter_shift)
        self.speed_fb_filtered += (speed_fb_rpm_instant - self.speed_fb_filtered) * alpha1

        # Zero-snap
        fb = self.speed_fb_filtered
        if abs(fb * (cpr * pp / 60.0)) < fp.speed_zero_snap:
            fb = 0.0

        # Stage 2:  speed_fb_ctrl += (fb - ctrl) >> shift
        alpha2 = 1.0 / (1 << fp.speed_ctrl_filter_shift)
        self.speed_fb_ctrl += (fb - self.speed_fb_ctrl) * alpha2

        fb_rpm = self.speed_fb_ctrl

        # Deadband
        if abs(speed_ref_rpm) < fp.speed_deadband_rpm:
            self.pi_speed.reset()
            self.speed_iq_cmd = 0.0
            self.speed_ref_active = 0.0
            return

        # Speed ref ramp
        ramp_step = fp.iq_slew_up  # simplified ramp
        self.speed_ref_active = self._slew(self.speed_ref_active,
                                           speed_ref_rpm, ramp_step)

        # Speed PI
        err = self.speed_ref_active - fb_rpm
        out_min = -fp.iq_limit
        out_max = fp.iq_limit
        # Allow braking for teaching (real fw has CTRL_SPD_BRAKE_ENABLE=0
        # which clamps out_min=0 for pos ref, but that prevents regulation)

        iq_pi = self.pi_speed.update(self.speed_ref_active, fb_rpm,
                                     fp.speed_kp, fp.speed_ki,
                                     fp.speed_shift, out_min, out_max)

        # Feedforward (kill when overspeeding, matching firmware)
        if ((speed_ref_rpm > fp.speed_deadband_rpm and fb_rpm >= speed_ref_rpm) or
            (speed_ref_rpm < -fp.speed_deadband_rpm and fb_rpm <= speed_ref_rpm)):
            self.speed_iq_ff = 0.0
        else:
            self.speed_iq_ff = speed_ref_rpm * fp.iq_ff_per_krpm / 1000.0

        iq_target = iq_pi + self.speed_iq_ff
        iq_target = max(out_min, min(out_max, iq_target))

        # Asymmetric slew
        step = fp.iq_slew_up if iq_target >= self.speed_iq_cmd else fp.iq_slew_down
        self.speed_iq_cmd = self._slew(self.speed_iq_cmd, iq_target, step)


# ============================================================================
# 4. Simulation engine
# ============================================================================


class SimulationEngine:
    """Run a complete FOC simulation and collect time histories."""

    def __init__(self, mp: MotorParams, fp: FOCParams):
        self.mp = mp
        self.fp = fp
        self.motor = MotorModel(mp)
        self.foc = FOCController(fp, mp)
        self._records: dict[str, list[float]] = {}

    def run(self, duration: float,
            id_ref_profile: Callable[[float], float] | None = None,
            iq_ref_profile: Callable[[float], float] | None = None,
            speed_mode: bool = False,
            speed_ref_rpm: float = 0.0,
            load_torque: float = 0.0) -> dict[str, np.ndarray]:
        """Run simulation for `duration` seconds.

        Args:
            duration: Simulation time in seconds.
            id_ref_profile: Function of time [s] → id_ref [counts].
            iq_ref_profile: Function of time [s] → iq_ref [counts].
            speed_mode: Enable cascaded speed+current loop.
            speed_ref_rpm: Speed reference in rpm (speed_mode only).
            load_torque: Constant load torque [Nm].
        """
        fp = self.fp
        motor = self.motor
        foc = self.foc

        motor.reset(rotor_locked=not speed_mode)
        foc.reset()

        num_ctrl_steps = int(duration / fp.dt_ctrl)
        record_every = max(1, num_ctrl_steps // 2000)  # ~2000 points for plotting
        record_step = 0

        keys = ["t", "i_d", "i_q", "i_d_ref", "i_q_ref",
                "v_d", "v_q", "v_d_lim", "v_q_lim", "v_limited",
                "v_d_phys", "v_q_phys", "valpha", "vbeta",
                "ialpha", "ibeta", "theta_16",
                "speed_rpm", "speed_ref_rpm",
                "speed_iq_cmd", "speed_iq_ff",
                "speed_fb_filtered", "speed_fb_ctrl",
                "pi_d_integral", "pi_q_integral",
                "i_u_phys", "i_v_phys", "i_w_phys"]
        records: dict[str, list[float]] = {k: [] for k in keys}

        # Pre-compute speed feedback in counts/s from motor mechanical speed
        cpr = 65536
        pole_pairs = self.mp.pole_pairs

        for step in range(num_ctrl_steps):
            t = step * fp.dt_ctrl

            # Determine current references
            if id_ref_profile is not None:
                id_ref = id_ref_profile(t)
            else:
                id_ref = 0.0

            if iq_ref_profile is not None:
                iq_ref = iq_ref_profile(t)
            else:
                iq_ref = 0.0

            # Speed feedback in raw counts/s (from motor mechanical speed)
            speed_fb_counts = (motor.omega_m / (2 * math.pi)) * cpr * pole_pairs

            # Phase currents in ADC counts
            # From motor i_d, i_q in Amps → phase currents via inverse Clarke+Park
            # i_a = i_α = i_d·cosθ - i_q·sinθ  (inverse Park)
            # i_b = -i_α/2 + i_β·√3/2  where i_β = i_d·sinθ + i_q·cosθ
            elec_rad = motor.theta_elec
            cos_t = math.cos(elec_rad)
            sin_t = math.sin(elec_rad)
            ialpha_phys = motor.i_d * cos_t - motor.i_q * sin_t
            ibeta_phys = motor.i_d * sin_t + motor.i_q * cos_t
            # Inverse Clarke
            i_u_phys = ialpha_phys
            i_v_phys = -0.5 * ialpha_phys + (math.sqrt(3) / 2) * ibeta_phys
            i_w_phys = -i_u_phys - i_v_phys

            # Convert physical amps → ADC counts
            a_to_count = fp.i_count_per_a
            i_u = i_u_phys * a_to_count
            i_v = i_v_phys * a_to_count
            i_w = i_w_phys * a_to_count

            # Run FOC control
            ctrl_out = foc.run_one_cycle(
                i_u, i_v, i_w,
                motor.theta_elec / pole_pairs,  # mechanical angle
                id_ref, iq_ref,
                speed_mode, speed_ref_rpm,
                speed_fb_counts)

            # Apply voltages to motor
            v_d_phys = ctrl_out["v_d_phys"]
            v_q_phys = ctrl_out["v_q_phys"]
            motor.run_one_control_cycle(v_d_phys, v_q_phys, load_torque)

            # Record
            if record_step == 0:
                speed_rpm_actual = motor.omega_m * 60.0 / (2 * math.pi)
                records["t"].append(t * 1000)  # ms
                records["i_d"].append(ctrl_out["i_d"])
                records["i_q"].append(ctrl_out["i_q"])
                records["i_d_ref"].append(ctrl_out["i_d_ref"])
                records["i_q_ref"].append(ctrl_out["i_q_ref"])
                records["v_d"].append(ctrl_out["v_d"])
                records["v_q"].append(ctrl_out["v_q"])
                records["v_d_lim"].append(ctrl_out["v_d_lim"])
                records["v_q_lim"].append(ctrl_out["v_q_lim"])
                records["v_limited"].append(ctrl_out["v_limited"])
                records["valpha"].append(ctrl_out["valpha"])
                records["vbeta"].append(ctrl_out["vbeta"])
                records["ialpha"].append(ctrl_out["ialpha"])
                records["ibeta"].append(ctrl_out["ibeta"])
                records["theta_16"].append(ctrl_out["theta_16"])
                records["speed_rpm"].append(speed_rpm_actual)
                records["speed_ref_rpm"].append(speed_ref_rpm)
                records["speed_iq_cmd"].append(ctrl_out["speed_iq_cmd"])
                records["speed_iq_ff"].append(ctrl_out["speed_iq_ff"])
                records["speed_fb_filtered"].append(ctrl_out["speed_fb_filtered"])
                records["speed_fb_ctrl"].append(ctrl_out["speed_fb_ctrl"])
                records["pi_d_integral"].append(ctrl_out["pi_d_integral"])
                records["pi_q_integral"].append(ctrl_out["pi_q_integral"])
                records["i_u_phys"].append(i_u_phys)
                records["i_v_phys"].append(i_v_phys)
                records["i_w_phys"].append(i_w_phys)
            record_step = (record_step + 1) % record_every

        return {k: np.array(v) for k, v in records.items()}


# ============================================================================
# 5. Scenario definitions
# ============================================================================

def _make_scenarios() -> dict[str, dict[str, Any]]:
    """Build the scenario catalog. Each entry is a dict of FOCParams overrides
    plus metadata: title, description, diagnostic (what to look for)."""
    base = FOCParams()

    return {
        "1. Well Tuned": {
            "title": "Well Tuned Baseline",
            "desc": "Correct Kp/Ki, zero aligned. The gold-standard response.",
            "diagnostic": (
                "EXPECTED: id ≈ 0 throughout, iq rises cleanly to iq_ref.\n"
                "Rise time ≈ 0.5 ms (τ_e ≈ 4.75 μs is near-instantaneous).\n"
                "Small overshoot, no oscillation. vd ≈ 0, vq ≈ R·iq_ref.\n"
                "This is what 'good tuning' looks like."
            ),
            "overrides": {"current_kp": 4, "current_ki": 4,
                          "zero_offset_deg": 0.0},
        },
        "2. Zero Offset 30°": {
            "title": "Zero Offset 30° (Mild Coupling)",
            "desc": "Electrical zero is off by 30°. id and iq partially swap roles.",
            "diagnostic": (
                "OBSERVE: id ≠ 0 even though id_ref = 0.\n"
                "WHY: Park transform uses wrong θ (+30°).\n"
                "Measured currents are rotated 30° vs physical.\n"
                "id_measured = i_d_real·cos(30°) + i_q_real·sin(30°)\n"
                "iq_measured = -i_d_real·sin(30°) + i_q_real·cos(30°)\n"
                "→ ~13% of iq_real leaks into measured id.\n"
                "Motor still produces torque but less efficiently.\n"
                "NOTE: Physical motor currents differ from measured values\n"
                "shown on the plot (which are controller-measured)."
            ),
            "overrides": {"current_kp": 4, "current_ki": 4,
                          "zero_offset_deg": 30.0},
        },
        "3. Zero Offset 90°": {
            "title": "Zero Offset 90° (Total Collapse)",
            "desc": "Electrical zero off by 90°. d and q axes completely swapped.",
            "diagnostic": (
                "CRITICAL: Plot shows id≈0, iq≈ref — controller looks happy.\n"
                "BUT: Park transform is 90° rotated, so:\n"
                "  id_measured = i_q_real  (physical q-axis current!)\n"
                "  iq_measured = -i_d_real (physical d-axis current!)\n"
                "The PI drives iq_measured→100 → physical i_d = -100 counts.\n"
                "Physical i_q ≈ 0 → motor produces NO torque.\n"
                "All current flows in d-axis (magnetizing, not torquing).\n"
                "CLASSIC CLUE: 'Controller looks fine but motor won't spin'\n"
                "LESSON: Zero offset hides from the controller — you must\n"
                "verify with external evidence (does the motor actually turn?)"
            ),
            "overrides": {"current_kp": 4, "current_ki": 4,
                          "zero_offset_deg": 90.0},
        },
        "4. Kp Too High": {
            "title": "Kp Too High (Oscillation)",
            "desc": "Proportional gain 6× too high. Current oscillates.",
            "diagnostic": (
                "OBSERVE: iq oscillates around iq_ref, vq bounces between ±v_limit.\n"
                "WHY: High Kp → large voltage for small error → overshoot.\n"
                "Next sample: large opposite error → opposite voltage → undershoot.\n"
                "This is proportional-only limit-cycle oscillation.\n"
                "Discrete stability limit: Kp < 22.5 (from Jury criterion).\n"
                "Current Kp=24 > 22.5 → formally unstable."
            ),
            "overrides": {"current_kp": 24, "current_ki": 0,
                          "zero_offset_deg": 0.0},
        },
        "5. Ki=0 (P Only)": {
            "title": "Ki=0 — Pure Proportional",
            "desc": "No integral term. Large steady-state error.",
            "diagnostic": (
                "OBSERVE: iq settles BELOW iq_ref. Steady-state error ≈ 80%!\n"
                "WHY: Without integral, the PI cannot accumulate error memory.\n"
                "Steady-state: v_q = Kp·error / 2^shift = R·i_q (Ohm's law).\n"
                "→ error = R·i_q_ref · 2^shift / Kp ≈ 4·100·8/4 = 800 counts!\n"
                "The P term alone cannot produce enough voltage to drive ref current.\n"
                "LESSON: Ki exists to eliminate steady-state error (final value theorem)."
            ),
            "overrides": {"current_kp": 4, "current_ki": 0,
                          "zero_offset_deg": 0.0},
        },
        "6. Ki Too High": {
            "title": "Ki Too High (Ringing)",
            "desc": "Integral gain 8× too high. Overshoot and ringing.",
            "diagnostic": (
                "OBSERVE: iq overshoots significantly, then rings 2-3 cycles.\n"
                "WHY: Large Ki accumulates integral fast → overshoot.\n"
                "Then integral must 'unwind' → undershoot → re-accumulate.\n"
                "The anti-windup clamps integral but the Ki gain is too aggressive\n"
                "for the plant. Dominant pole near z=0.7 → underdamped.\n"
                "FIX: Reduce Ki or increase Kp to maintain Kp/Ki ratio."
            ),
            "overrides": {"current_kp": 4, "current_ki": 32,
                          "zero_offset_deg": 0.0},
        },
        "7. Wrong L (10×)": {
            "title": "Wrong Inductance — PI Mismatch",
            "desc": "PI designed for L=190μH but motor is 19μH. Sluggish response.",
            "diagnostic": (
                "OBSERVE: Response is very slow (~5 ms to reach ref).\n"
                "WHY: The PI bandwidth formula is ω_c = Kp/L.\n"
                "If you tune PI assuming L=190μH but real L=19μH:\n"
                "The PI zero (Ki/Kp = R/L_estimated) is 10× slower than needed.\n"
                "→ Integral time constant = L_estimated/R = 47.5 μs (but motor\n"
                "settles in 4.75 μs). The PI is fighting the last war.\n"
                "LESSON: PI tuning depends critically on accurate L and R."
            ),
            "overrides": {"current_kp": 1, "current_ki": 1,
                          "zero_offset_deg": 0.0},
        },
        "8. Voltage Saturation": {
            "title": "Voltage Saturation & Anti-Windup",
            "desc": "iq_ref exceeds what Vbus can deliver. Anti-windup prevents "
                    "integral runaway.",
            "diagnostic": (
                "OBSERVE: vq hits v_limit (887), iq plateaus below ref.\n"
                "v_limited flag toggles. iq cannot reach ref — physics limit.\n"
                "Anti-windup: when v_limited=1, integral stops accumulating.\n"
                "Without anti-windup, integral would grow unbounded ('windup'),\n"
                "causing massive overshoot when ref drops.\n"
                "CHECK: Is integral still reasonable? → Anti-windup is working.\n"
                "FIX: Reduce iq_ref or increase Vbus (hardware change)."
            ),
            "overrides": {"current_kp": 4, "current_ki": 4,
                          "zero_offset_deg": 0.0,
                          "iq_limit": 1000},  # high ref triggers saturation
        },
        "9. Heavy Speed Filter": {
            "title": "Speed Oscillation (Heavy Filter)",
            "desc": "Two-stage speed filter (τ≈40ms) causes delay-induced oscillation.",
            "diagnostic": (
                "OBSERVE: Speed oscillates ~±50 rpm around ref at ~2-5 Hz.\n"
                "WHY: Combined filter τ ≈ 40 ms → phase lag at loop crossover.\n"
                "speed_fb_ctrl (used by PI) lags real speed by ~40 ms.\n"
                "PI reacts to old data → over-corrects → oscillation.\n"
                "FIX: Reduce CTRL_SPD_CTRL_FILTER_SHIFT from 4→1.\n"
                "Or use speed_fb (1st-stage only) for PI feedback."
            ),
            "overrides": {"current_kp": 4, "current_ki": 4,
                          "speed_kp": 16, "speed_ki": 4,
                          "speed_filter_shift": 2,
                          "speed_ctrl_filter_shift": 5,  # heavy → τ≈64ms
                          "zero_offset_deg": 0.0},
        },
        "10. Speed Kp Too High": {
            "title": "Speed Kp Too High",
            "desc": "Aggressive speed PI causes iq_cmd oscillation → speed hunting.",
            "diagnostic": (
                "OBSERVE: iq_cmd swings widely, speed oscillates at ~20-50 Hz.\n"
                "WHY: Speed Kp too large → small speed error → large iq swing.\n"
                "The current loop (inner) tracks iq_cmd faithfully, but iq_cmd\n"
                "itself is oscillating due to the outer speed loop gain.\n"
                "This is a classic cascaded-loop instability.\n"
                "FIX: Reduce speed Kp, or reduce current loop bandwidth first."
            ),
            "overrides": {"current_kp": 4, "current_ki": 4,
                          "speed_kp": 50, "speed_ki": 10,
                          "speed_filter_shift": 1,
                          "speed_ctrl_filter_shift": 2,
                          "zero_offset_deg": 0.0},
        },
        "M. Mystery Fault": {
            "title": "??? Mystery Fault ???",
            "desc": "Something is wrong. Diagnose from the waveforms.",
            "diagnostic": (
                "??? — Click 'Reveal Fault' to see what was injected.\n"
                "Look at: id vs id_ref, iq vs iq_ref, v_limited, speed ripple.\n"
                "Common patterns:\n"
                "  id≠0 → zero offset or phase mapping issue\n"
                "  oscillation → Kp/Ki too high\n"
                "  steady error → Ki too low\n"
                "  slow response → wrong L or low Kp\n"
                "  speed oscillation → filter delay or speed Kp too high"
            ),
            "overrides": {},  # filled at runtime by _inject_mystery()
            "mystery": True,
        },
    }


SCENARIOS = _make_scenarios()

# Possible mystery faults
_MYSTERY_FAULTS = [
    {"zero_offset_deg": 25.0},
    {"zero_offset_deg": -65.0},
    {"current_kp": 20, "current_ki": 0},
    {"current_kp": 1, "current_ki": 1},
    {"current_kp": 4, "current_ki": 25},
    {"speed_kp": 40, "speed_ki": 8,
     "speed_filter_shift": 2, "speed_ctrl_filter_shift": 4},
    {"speed_ctrl_filter_shift": 6},
    {"deadtime_comp_missing": True},
]


def _inject_mystery() -> dict[str, Any]:
    """Pick a random fault and return the FOCParams overrides."""
    fault = random.choice(_MYSTERY_FAULTS)
    override = dict(fault)
    # Ensure baseline params for non-faulted fields
    override.setdefault("current_kp", 4)
    override.setdefault("current_ki", 4)
    override.setdefault("zero_offset_deg", 0.0)
    override.setdefault("speed_kp", 16)
    override.setdefault("speed_ki", 4)
    override.setdefault("speed_filter_shift", 2)
    override.setdefault("speed_ctrl_filter_shift", 4)
    override.setdefault("deadtime_comp_missing", False)
    return override


# ============================================================================
# 6. Interactive visualization
# ============================================================================


class FOCVisualizer:
    """Matplotlib-based interactive FOC simulator UI."""

    def __init__(self):
        self.mp = MotorParams()
        self.fp = FOCParams()
        self.engine = SimulationEngine(self.mp, self.fp)

        # Current state
        self.current_scenario = "1. Well Tuned"
        self.speed_mode = False
        self.mystery_revealed = False
        self._last_mystery_override: dict[str, Any] = {}
        self._last_data: dict[str, np.ndarray] = {}
        self._last_speed_rpm: float = 500.0

        # Build UI
        self._build_ui()

    # ---- UI construction ----------------------------------------------------

    def _build_ui(self):
        self.fig = plt.figure("FOC Teaching Simulator", figsize=(16, 10))
        self.fig.set_tight_layout(False)

        gs = GridSpec(3, 3, figure=self.fig,
                      left=0.05, right=0.72, top=0.95, bottom=0.08,
                      hspace=0.45, wspace=0.35,
                      height_ratios=[1, 1, 1])

        # Plot panels
        self.ax_idq = self.fig.add_subplot(gs[0, :2])
        self.ax_idq.set_title("dq Current Response", fontweight="bold")
        self.ax_idq.set_xlabel("Time [ms]")
        self.ax_idq.set_ylabel("Current [counts]")
        self.ax_idq.grid(True, alpha=0.3)

        self.ax_vdq = self.fig.add_subplot(gs[1, :2])
        self.ax_vdq.set_title("dq Voltage Output", fontweight="bold")
        self.ax_vdq.set_xlabel("Time [ms]")
        self.ax_vdq.set_ylabel("Voltage [counts]")
        self.ax_vdq.grid(True, alpha=0.3)

        self.ax_speed = self.fig.add_subplot(gs[2, :2])
        self.ax_speed.set_title("Speed Response", fontweight="bold")
        self.ax_speed.set_xlabel("Time [ms]")
        self.ax_speed.set_ylabel("Speed [rpm] / iq [counts]")
        self.ax_speed.grid(True, alpha=0.3)

        # Diagnostic text panel
        self.ax_diag = self.fig.add_subplot(gs[:, 2])
        self.ax_diag.axis("off")
        self._diag_text = self.ax_diag.text(
            0.02, 0.98, "", transform=self.ax_diag.transAxes,
            fontfamily="monospace", fontsize=9,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

        # ---- Widgets (placed outside the grid area) ----

        # Scenario radio buttons
        ax_radio = self.fig.add_axes([0.74, 0.55, 0.20, 0.40])
        scenario_names = [k for k in SCENARIOS if not SCENARIOS[k].get("mystery")]
        n = len(scenario_names)
        self._radio = RadioButtons(ax_radio, scenario_names,
                                   active=0,
                                   label_props={"fontsize": [8] * n},
                                   radio_props={"s": [10] * n})
        self._radio.on_clicked(self._on_scenario)

        # Mode toggle button
        ax_mode = self.fig.add_axes([0.74, 0.46, 0.09, 0.04])
        self._btn_mode = Button(ax_mode, "Speed Mode: OFF",
                                color="lightgray", hovercolor="lightblue")
        self._btn_mode.label.set_fontsize(8)
        self._btn_mode.on_clicked(self._on_toggle_mode)

        # Mystery fault button
        ax_mystery = self.fig.add_axes([0.85, 0.46, 0.09, 0.04])
        self._btn_mystery = Button(ax_mystery, "Mystery Fault",
                                   color="lightcoral", hovercolor="red")
        self._btn_mystery.label.set_fontsize(8)
        self._btn_mystery.on_clicked(self._on_mystery)

        # Reveal button
        ax_reveal = self.fig.add_axes([0.85, 0.40, 0.09, 0.04])
        self._btn_reveal = Button(ax_reveal, "Reveal Fault",
                                  color="lightyellow", hovercolor="yellow")
        self._btn_reveal.label.set_fontsize(8)
        self._btn_reveal.on_clicked(self._on_reveal)

        # ---- Parameter sliders ----
        slider_specs = [
            ("Kp",       0.74, 0.34, 0, 30, 4,  0, "%d"),
            ("Ki",       0.74, 0.30, 0, 32, 4,  0, "%d"),
            ("L [μH]",   0.74, 0.26, 1, 200, 19, 0, "%d"),
            ("R [Ω]",    0.74, 0.22, 0.5, 10, 4.0, 1, "%.1f"),
            ("Zero Err°", 0.74, 0.18, -180, 180, 0, 0, "%d"),
            ("iq_ref",   0.74, 0.14, 0, 500, 100, 0, "%d"),
            ("Spd Kp",   0.74, 0.10, 0, 50, 16, 0, "%d"),
            ("Spd Ref",  0.74, 0.06, 0, 3000, 500, 0, "%d"),
        ]
        self._sliders: dict[str, Slider] = {}
        for label, x, y, vmin, vmax, vinit, decimals, fmt in slider_specs:
            ax_s = self.fig.add_axes([x, y, 0.18, 0.025])
            sl = Slider(ax_s, label, vmin, vmax, valinit=vinit,
                        valfmt=fmt if decimals == 0 else f"%.{decimals}f")
            sl.label.set_fontsize(8)
            sl.valtext.set_fontsize(7)
            sl.on_changed(self._make_slider_callback(label))
            self._sliders[label] = sl

        self.fig.canvas.mpl_connect("resize_event", self._on_resize)

    def _make_slider_callback(self, name: str):
        def callback(val):
            self._apply_sliders()
            self._rerun()
        return callback

    def _apply_sliders(self):
        """Read slider values into fp."""
        self.fp.current_kp = int(self._sliders["Kp"].val)
        self.fp.current_ki = int(self._sliders["Ki"].val)
        self.mp.L = float(self._sliders["L [μH]"].val) * 1e-6
        self.mp.R = float(self._sliders["R [Ω]"].val)
        self.fp.zero_offset_deg = float(self._sliders["Zero Err°"].val)
        self.fp.speed_kp = int(self._sliders["Spd Kp"].val)
        self._last_speed_rpm = float(self._sliders["Spd Ref"].val)

    def _set_sliders_from_fp(self):
        """Update slider positions to match fp values."""
        self._sliders["Kp"].set_val(self.fp.current_kp)
        self._sliders["Ki"].set_val(self.fp.current_ki)
        self._sliders["L [μH]"].set_val(self.mp.L * 1e6)
        self._sliders["R [Ω]"].set_val(self.mp.R)
        self._sliders["Zero Err°"].set_val(self.fp.zero_offset_deg)
        self._sliders["Spd Kp"].set_val(self.fp.speed_kp)
        # iq_ref is set by scenario

    # ---- Callbacks ----------------------------------------------------------

    def _on_scenario(self, label: str):
        self.current_scenario = label
        self.mystery_revealed = False
        info = SCENARIOS[label]
        overrides = info.get("overrides", {}).copy()

        if info.get("mystery"):
            overrides = _inject_mystery()
            self._last_mystery_override = overrides
            self.mystery_revealed = False

        # Apply overrides
        self.fp = FOCParams()
        self.mp = MotorParams()
        for k, v in overrides.items():
            if hasattr(self.fp, k):
                setattr(self.fp, k, v)
            elif hasattr(self.mp, k):
                setattr(self.mp, k, v)
        self.engine = SimulationEngine(self.mp, self.fp)
        self._set_sliders_from_fp()
        self._update_diagnostic(info["diagnostic"])
        self._rerun()

    def _on_toggle_mode(self, event):
        self.speed_mode = not self.speed_mode
        if self.speed_mode:
            self._btn_mode.label.set_text("Speed Mode: ON")
            self._btn_mode.color = "lightgreen"
        else:
            self._btn_mode.label.set_text("Speed Mode: OFF")
            self._btn_mode.color = "lightgray"
        self._rerun()

    def _on_mystery(self, event):
        """Inject a mystery fault into the current scenario."""
        self.mystery_revealed = False
        overrides = _inject_mystery()
        self._last_mystery_override = overrides
        for k, v in overrides.items():
            if hasattr(self.fp, k):
                setattr(self.fp, k, v)
            elif hasattr(self.mp, k):
                setattr(self.mp, k, v)
        self._set_sliders_from_fp()
        self._update_diagnostic(SCENARIOS["M. Mystery Fault"]["diagnostic"])
        self._rerun()

    def _on_reveal(self, event):
        """Reveal the mystery fault."""
        if not self._last_mystery_override:
            self._update_diagnostic("No mystery fault active. Click 'Mystery Fault' first.")
            return
        self.mystery_revealed = True
        lines = ["FAULT REVEALED — the following was changed:"]
        for k, v in self._last_mystery_override.items():
            default = getattr(FOCParams(), k, "?")
            lines.append(f"  {k}: default={default} → FAULT={v}")
        self._update_diagnostic("\n".join(lines))

    def _on_resize(self, event):
        self.fig.canvas.draw_idle()

    # ---- Simulation ---------------------------------------------------------

    def _rerun(self):
        """Run simulation and update plots."""
        fp = self.fp
        iq_ref_val = int(self._sliders["iq_ref"].val)
        speed_ref = self._last_speed_rpm

        if self.speed_mode:
            duration = 1.0  # longer for speed loop dynamics
            data = self.engine.run(
                duration,
                id_ref_profile=lambda t: 0.0,
                speed_mode=True,
                speed_ref_rpm=speed_ref,
                load_torque=0.0)
        else:
            duration = 0.02  # 20 ms for current loop
            # Step: iq_ref from 0 to ref at t=1ms
            def iq_profile(t):
                if t < 0.001:
                    return 0.0
                return float(iq_ref_val)

            data = self.engine.run(
                duration,
                id_ref_profile=lambda t: 0.0,
                iq_ref_profile=iq_profile)

        self._last_data = data
        self._plot(data)

    # ---- Plotting -----------------------------------------------------------

    def _plot(self, data: dict[str, np.ndarray]):
        t = data["t"]
        fp = self.fp

        # --- dq Currents ---
        self.ax_idq.clear()
        self.ax_idq.set_title("dq Current Response", fontweight="bold")
        self.ax_idq.plot(t, data["i_d_ref"], "k--", alpha=0.5, lw=1, label="id_ref")
        self.ax_idq.plot(t, data["i_d"], "b-", lw=1.5, label="id")
        self.ax_idq.plot(t, data["i_q_ref"], "k--", alpha=0.5, lw=1, label="iq_ref")
        self.ax_idq.plot(t, data["i_q"], "r-", lw=1.5, label="iq")
        self.ax_idq.set_ylabel("Current [counts]")
        self.ax_idq.legend(loc="upper right", fontsize=7, ncol=2)
        self.ax_idq.grid(True, alpha=0.3)

        # Annotate with key metrics
        if len(data["i_q"]) > 10:
            iq_final = data["i_q"][-10:].mean()
            iq_ref_final = data["i_q_ref"][-10:].mean()
            ss_err = iq_ref_final - iq_final
            self.ax_idq.text(0.98, 0.5,
                             f"iq ss: {iq_final:.0f}\n"
                             f"ss err: {ss_err:.0f}",
                             transform=self.ax_idq.transAxes,
                             fontsize=8, verticalalignment="center",
                             horizontalalignment="right",
                             bbox=dict(boxstyle="round", facecolor="white", alpha=0.7))

        # --- dq Voltages ---
        self.ax_vdq.clear()
        self.ax_vdq.set_title("dq Voltage Output", fontweight="bold")
        self.ax_vdq.plot(t, data["v_d_lim"], "b-", lw=1.5, label="vd")
        self.ax_vdq.plot(t, data["v_q_lim"], "r-", lw=1.5, label="vq")
        self.ax_vdq.axhline(fp.v_limit, color="gray", ls="--", lw=1, alpha=0.5)
        self.ax_vdq.axhline(-fp.v_limit, color="gray", ls="--", lw=1, alpha=0.5)
        self.ax_vdq.set_ylabel("Voltage [counts]")
        self.ax_vdq.legend(loc="upper right", fontsize=7)
        self.ax_vdq.grid(True, alpha=0.3)

        # v_limited indicator
        if np.any(data["v_limited"] > 0.5):
            vlim_times = t[data["v_limited"] > 0.5]
            for vt in vlim_times[::max(1, len(vlim_times) // 20)]:
                self.ax_vdq.axvspan(vt, vt + 0.05, color="red", alpha=0.15)

        # --- Speed ---
        self.ax_speed.clear()
        self.ax_speed.set_title("Speed Response", fontweight="bold")
        if self.speed_mode:
            ax2 = self.ax_speed.twinx()
            self.ax_speed.plot(t, data["speed_ref_rpm"], "k--", alpha=0.5, lw=1,
                               label="ω ref [rpm]")
            self.ax_speed.plot(t, data["speed_rpm"], "b-", lw=1.5, label="ω [rpm]")
            self.ax_speed.plot(t, data["speed_fb_ctrl"], "g-", lw=1, alpha=0.7,
                               label="ω ctrl (filtered)")
            self.ax_speed.set_ylabel("Speed [rpm]")
            self.ax_speed.set_ylim(-50, max(100, data["speed_ref_rpm"].max() * 1.5))
            self.ax_speed.legend(loc="upper left", fontsize=7)
            ax2.plot(t, data["speed_iq_cmd"], "r-", lw=1, alpha=0.7, label="iq cmd")
            ax2.set_ylabel("iq cmd [counts]", color="red")
            ax2.tick_params(axis="y", labelcolor="red")
            ax2.legend(loc="upper right", fontsize=7)
        else:
            self.ax_speed.plot(t, data["speed_rpm"], "b-", lw=1.5)
            self.ax_speed.set_ylabel("Speed [rpm]")
            self.ax_speed.text(0.5, 0.5, "Speed loop disabled.\nToggle 'Speed Mode' to enable.",
                               transform=self.ax_speed.transAxes,
                               ha="center", va="center", fontsize=10, color="gray")
        self.ax_speed.set_xlabel("Time [ms]")
        self.ax_speed.grid(True, alpha=0.3)

        self.fig.canvas.draw_idle()

    # ---- Diagnostics --------------------------------------------------------

    def _update_diagnostic(self, text: str):
        self._diag_text.set_text(text)

    # ---- Run ----------------------------------------------------------------

    def show(self):
        """Display the UI and start the interactive loop."""
        # Apply initial scenario
        self._on_scenario(self.current_scenario)
        # Show
        plt.show()


# ============================================================================
# 7. Main
# ============================================================================


def main():
    print("=" * 60)
    print("  FOC Teaching Simulator")
    print("  Motor: 19 μH, 4 Ω, 2.16 V/krpm, 4 pole pairs")
    print("  PWM: 20 kHz, SVPWM, 12-bit ADC")
    print("=" * 60)
    print()
    print("Controls:")
    print("  Left panel:  select teaching scenarios")
    print("  Right panel: adjust parameters (Kp, Ki, L, R, zero offset...)")
    print("  'Speed Mode' toggle: switch between current-only and cascaded")
    print("  'Mystery Fault': random misconfiguration — you diagnose!")
    print("  'Reveal Fault': show what was wrong")
    print()
    print("Close the figure window to exit.")
    print()

    viz = FOCVisualizer()
    viz.show()


if __name__ == "__main__":
    main()
