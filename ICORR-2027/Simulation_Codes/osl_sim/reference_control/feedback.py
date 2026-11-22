"""Shared derivative-location variants for diagnostics and production gait.

A delegates its command verbatim to CurrentPID. B replaces only its derivative
term, after running that same PID for the knee-side P/I terms and history. The
reference derivative is the control-sample backward difference, NOT the spline
velocity channel. B uses instantaneous pre-step actuator-output velocity.
"""
from __future__ import annotations

import math

from .current import CurrentPID
from .gait import GaitCurrentSource


def referred_velocity(theta_a_dot, belt_ratio, measurement_sign):
    """Post-planetary / pre-belt rad/s -> controller knee degrees/s."""
    if belt_ratio <= 0 or measurement_sign not in (-1, 1):
        raise ValueError("Positive belt ratio and +/-1 measurement sign required")
    return measurement_sign * math.degrees(theta_a_dot / belt_ratio)


class FeedbackLocationPID:
    def __init__(self, gains, limits, mode, integral_limit_deg_s=None):
        if mode not in ('A', 'B'):
            raise ValueError('Diagnostic requires A/B mode')
        self.base = CurrentPID(gains, limits, derivative_on='error',
                               integral_limit_deg_s=integral_limit_deg_s)
        self.mode = mode
        self.reset()

    def reset(self):
        self.base.reset()
        self.prev_ref = self.prev_meas = None
        self.actuator_velocity_deg_s = 0.0
        self.last = self.base.last.copy()
        self.signals = dict(reference_derivative_deg_s=0.0,
                            joint_difference_velocity_deg_s=0.0,
                            derivative_feedback_velocity_deg_s=0.0,
                            reference_derivative_current_A=0.0,
                            measurement_derivative_current_A=0.0,
                            derivative_decomposition_residual_A=0.0)

    def update(self, ref_deg, meas_deg, dt):
        command = self.base.update(ref_deg, meas_deg, dt)
        self.last = self.base.last.copy()
        used = self.prev_ref is not None
        vr = (ref_deg - self.prev_ref) / dt if used else 0.0
        vj = (meas_deg - self.prev_meas) / dt if used else 0.0
        vf = vj if self.mode == 'A' else self.actuator_velocity_deg_s
        kd = self.base.gains.kd_A_per_deg_per_s
        dr = kd * vr if used else 0.0
        dm = -kd * vf if used else 0.0
        if self.mode == 'B':
            d = kd * (vr - vf) if used else 0.0
            raw = self.last['p_A'] + self.last['i_A'] + d
            clip = self.base.limits.sim_clip_A
            clipped = max(-clip, min(clip, raw))
            sign = self.base.gains.encoder_to_motor_sign
            command = sign * clipped
            self.last.update(d_A=d, raw_A=raw, raw_signed_A=sign * raw,
                             cmd_A=command, saturated=clipped != raw)
        self.signals = dict(reference_derivative_deg_s=vr,
                            joint_difference_velocity_deg_s=vj,
                            derivative_feedback_velocity_deg_s=vf,
                            reference_derivative_current_A=dr,
                            measurement_derivative_current_A=dm,
                            derivative_decomposition_residual_A=self.last['d_A']-dr-dm)
        self.prev_ref, self.prev_meas = ref_deg, meas_deg
        return command


class FeedbackLocationSource(GaitCurrentSource):
    """Reuse the production schedule, current loop and conversion unchanged."""
    def __init__(self, cfg, audit, ref_rad, steps_per_update, layer, mode):
        if cfg.derivative_on != 'error' or layer.p.n_t != audit.belt_ratio:
            raise ValueError('Expected error derivative and identical belt ratios')
        self.layer = layer
        self.mode = mode
        super().__init__(cfg, audit, ref_rad, steps_per_update)
        self.pid = FeedbackLocationPID(cfg.gains, cfg.limits, mode, cfg.integral_limit_deg_s)
        self.reset()

    def __call__(self, k, theta_j, theta_j_dot):
        self.pid.actuator_velocity_deg_s = referred_velocity(
            self.layer.theta_a_dot, self.layer.p.n_t, self.cfg.measurement_sign)
        result = super().__call__(k, theta_j, theta_j_dot)
        self.last.update(self.pid.signals)
        self.last.update(pre_step_knee_rad=float(theta_j),
                         pre_step_knee_velocity_rad_s=float(theta_j_dot),
                         pre_step_actuator_rad=self.layer.theta_a,
                         pre_step_actuator_velocity_rad_s=self.layer.theta_a_dot,
                         controller_updated=int(max(k, 0) % self.steps_per_update == 0))
        return result


def referred_angle(theta_a, belt_ratio, measurement_sign):
    """Actuator OUTPUT shaft rad -> knee-equivalent controller degrees."""
    if belt_ratio <= 0 or measurement_sign not in (-1, 1):
        raise ValueError('Positive belt ratio and +/-1 measurement sign required')
    return measurement_sign * math.degrees(theta_a / belt_ratio)


class ActuatorDifferencePID(FeedbackLocationPID):
    """C: supply an angle difference to the existing B derivative arithmetic.

    A/B classes are unmodified. The inherited B calculation preserves reference
    differencing, suppression, signs and clipping. Only its feedback input differs.
    """
    def __init__(self, gains, limits, integral_limit_deg_s=None):
        super().__init__(gains, limits, 'B', integral_limit_deg_s)

    def reset(self):
        super().reset()
        self.actuator_angle_deg = 0.0
        self.prev_actuator_angle_deg = None

    def update(self, ref_deg, meas_deg, dt):
        if dt <= 0:
            raise ValueError('Controller period must be positive')
        previous = self.prev_actuator_angle_deg
        velocity = ((self.actuator_angle_deg - previous) / dt
                    if previous is not None else 0.0)
        # Parent source may supply instantaneous velocity; C explicitly replaces
        # it here, only when the inherited scheduler calls update.
        self.actuator_velocity_deg_s = velocity
        command = super().update(ref_deg, meas_deg, dt)
        self.signals.update(actuator_equivalent_sample_deg=self.actuator_angle_deg,
                            actuator_difference_velocity_deg_s=velocity,
                            controller_period_s=float(dt))
        self.prev_actuator_angle_deg = self.actuator_angle_deg
        return command


class ActuatorDifferenceSource(FeedbackLocationSource):
    """C uses the same scheduler/current conversion and pre-step logging as A/B."""
    def __init__(self, cfg, audit, ref_rad, steps_per_update, layer):
        super().__init__(cfg, audit, ref_rad, steps_per_update, layer, 'B')
        self.mode = 'C'
        self.pid = ActuatorDifferencePID(cfg.gains, cfg.limits, cfg.integral_limit_deg_s)
        self.reset()

    def __call__(self, k, theta_j, theta_j_dot):
        if max(k, 0) % self.steps_per_update == 0:
            self.pid.actuator_angle_deg = referred_angle(
                self.layer.theta_a, self.layer.p.n_t, self.cfg.measurement_sign)
        return super().__call__(k, theta_j, theta_j_dot)
