#!/usr/bin/env python3
"""
joint_current_control.py -- per-joint current-domain control, shared by knee and ankle.

WHY THIS MODULE EXISTS
    `oslbench/sim/ankle.py` implements the physical ankle rig's architecture
    and is, correctly, full of ankle facts: the Vari-Flex foot geometry, the encoder's
    reported plantarflexion polarity, the ankle's own ROM.  The knee needs the SAME
    software -- degree-domain PD, amps out, first-order current loop, Option-C belt --
    with DIFFERENT per-joint facts.

    So the reusable parts (`CurrentPID`, `PIDGains`, `CurrentLimits`, `CurrentLoopModel`,
    `ConversionAudit`) are imported from there rather than copied, and this module adds
    only the per-joint CONFIGURATION and the adapter that drives the existing
    `DrivetrainBenchSimulation`.

    No control law and no drivetrain equation is restated here.

THE ONE THING MOST LIKELY TO BE GOT WRONG
    The knee and the ankle do NOT share a measurement-sign convention, and the reason is
    not cosmetic:

      ANKLE  the controller reads a PHYSICAL EXTERNAL ENCODER whose positive direction
             is a hardware fact.  `mjcf_q_to_encoder_sign = -1` is derived from the foot
             geometry (see the ankle module), and the hardware script's own
             `ENCODER_TO_MOTOR_SIGN = -1` is a separate, controller-internal quantity.

      KNEE   in THIS experiment both the reference and the measurement are MuJoCo
             generalized coordinates -- the AB19 CSV is a knee-angle curve in the same
             frame the model uses.  There is no physical knee encoder in the loop at
             all, so the measurement sign is +1 BY CONSTRUCTION, not by derivation.
             The motor sign is then fixed by requiring negative feedback in the model,
             which is a SIMULATION WIRING choice and is recorded as one.

    Model-frame signs remain simulation wiring. Named ankle-derived and supplied-knee
    controller profiles below select tuning, not the physical joint or encoder frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .current import ConversionAudit, CurrentLimits, CurrentLoopModel, CurrentPID, PIDGains

__all__ = ["JointCurrentConfig", "KNEE_GAIT_CONFIG", "GaitCurrentSource",
           "GAIN_SOURCE_NOTE", "ANKLE_CONTROLLER_CONFIG", "KNEE_CONTROLLER_CONFIG",
           "resolve_controller_settings"]

GAIN_SOURCE_NOTE = ("ankle step-test script; provisionally transferred to knee; "
                    "not hardware-confirmed for knee")


@dataclass(frozen=True)
class ControllerSettings:
    """Tuning profiles for the common knee gait runner, not joint/encoder mappings."""
    kp: float
    ki: float
    kd: float
    control_hz: float
    kt: str
    feedback_mode: str
    integral_limit_deg_s: float | None
    provenance: str


ANKLE_CONTROLLER_CONFIG = ControllerSettings(.75, 0., .03, 1000., 'manual_rotor',
    'legacy', None, GAIN_SOURCE_NOTE)
KNEE_CONTROLLER_CONFIG = ControllerSettings(1.5, .15, .06, 200., 'script',
    'actuator-difference', 50., 'Preetam supplied knee PID gains/integral limit/rate; '
    'C derivative is a simulation adaptation; output KT=0.5701 is provisional')


def resolve_controller_settings(args):
    """Explicit caller overrides win; omitted fields come from the selected profile."""
    name = getattr(args, 'controller', 'ankle')
    if name not in ('ankle', 'knee'):
        raise ValueError('Controller must be ankle or knee')
    profile = ANKLE_CONTROLLER_CONFIG if name == 'ankle' else KNEE_CONTROLLER_CONFIG
    args.controller = name
    for key in ('kp', 'ki', 'kd', 'control_hz', 'kt', 'feedback_mode', 'integral_limit_deg_s'):
        if getattr(args, key, None) is None:
            setattr(args, key, getattr(profile, key))
    if not math.isfinite(args.control_hz) or args.control_hz <= 0:
        raise ValueError('Controller rate must be finite and positive')
    for key in ('kp', 'ki', 'kd'):
        if not math.isfinite(getattr(args, key)):
            raise ValueError('Gains must be finite')
    return profile


def make_gait_source(cfg, audit, ref_rad, steps_per_update, layer, mode='legacy'):
    """Select the verified C adapter without changing the legacy scheduler or law."""
    if mode == 'legacy':
        return GaitCurrentSource(cfg, audit, ref_rad, steps_per_update)
    if mode == 'actuator-difference':
        # Local import avoids the diagnostic adapter's GaitCurrentSource dependency.
        from .feedback import ActuatorDifferenceSource
        return ActuatorDifferenceSource(cfg, audit, ref_rad, steps_per_update, layer)
    raise ValueError('Unknown feedback mode: '+str(mode))


@dataclass
class JointCurrentConfig:
    """Everything that differs between joints, in one place with its provenance.

    Deliberately a dataclass of plain facts rather than a class hierarchy: the whole
    point is that a reader can see, at a glance, which numbers belong to which joint and
    where each came from.
    """

    joint: str                                  # "knee" or "ankle"
    gains: PIDGains
    limits: CurrentLimits
    measurement_sign: int                       # model coordinate -> controller degrees
    measurement_frame: str                      # what the controller is reading
    gain_source: str
    sign_source: str
    derivative_on: str = "error"
    current_loop_tau_s: float = 0.0
    notes: str = ""
    integral_limit_deg_s: float | None = None

    def __post_init__(self):
        if self.measurement_sign not in (-1, 1):
            raise ValueError(f"measurement_sign must be +/-1, got {self.measurement_sign}")
        if self.derivative_on not in ("error", "measurement"):
            raise ValueError("derivative_on must be 'error' or 'measurement'")

    @property
    def loop_product(self) -> int:
        """measurement_sign * motor_sign.  Must be +1 for negative feedback."""
        return int(self.measurement_sign) * int(self.gains.encoder_to_motor_sign)

    @property
    def negative_feedback(self) -> bool:
        return self.loop_product > 0

    def to_controller_deg(self, q_rad: float) -> float:
        return self.measurement_sign * math.degrees(float(q_rad))

    def to_model_rad(self, deg: float) -> float:
        return math.radians(float(deg) / self.measurement_sign)

    def describe(self) -> list[str]:
        g = self.gains
        return [
            f"joint            {self.joint}",
            f"Kp               {g.kp_A_per_deg:g} A/deg",
            f"Ki               {g.ki_A_per_deg_s:g} A/(deg.s)",
            f"Kd               {g.kd_A_per_deg_per_s:g} A/(deg/s)",
            f"gain_source      {self.gain_source}",
            f"integral limit   {self.integral_limit_deg_s} degree-seconds (None: legacy current-derived limit)",
            f"measurement      {self.measurement_frame}",
            f"measurement_sign {self.measurement_sign:+d}",
            f"motor_sign       {g.encoder_to_motor_sign:+d}",
            f"sign_source      {self.sign_source}",
            f"loop product     {self.loop_product:+d} -> "
            f"{'NEGATIVE feedback' if self.negative_feedback else 'POSITIVE FEEDBACK'}",
            f"derivative on    {self.derivative_on}",
            f"current clip     {self.limits.sim_clip_A:g} A (simulation only)",
            f"current loop     tau_i = {self.current_loop_tau_s * 1e3:g} ms",
        ]


# Historical ankle-derived tuning applied to the knee model. Preserve old imports.
ANKLE_DERIVED_GAIT_CONFIG = JointCurrentConfig(
    joint="knee",
    # Kp/Kd are the ANKLE script's values, used here for an exploratory gait run
    # at those numbers.  They are not knee gains.
    gains=PIDGains(kp_A_per_deg=0.75, ki_A_per_deg_s=0.0, kd_A_per_deg_per_s=0.03,
                   encoder_to_motor_sign=+1),
    limits=CurrentLimits(),                     # 60 A sim clip, as the ankle script
    measurement_sign=+1,
    measurement_frame=("MuJoCo knee generalized coordinate, in degrees. The AB19 "
                       "reference is a knee-angle curve in the SAME frame, so no "
                       "encoder convention enters -- the sign is +1 by construction."),
    gain_source=GAIN_SOURCE_NOTE,
    sign_source=("measurement_sign +1 BY CONSTRUCTION (reference and measurement share "
                 "the model frame). motor_sign +1 chosen so the loop is negative "
                 "feedback, VERIFIED against the model's own torque chain: +I_q -> "
                 "+tau_m -> theta_s < 0 -> +tau_j -> knee flexes (+qpos). A SIMULATION "
                 "WIRING fact, NOT a transfer of the ankle's -1 and NOT a claim about "
                 "physical knee hardware."),
    notes="Historical ankle-derived knee gait configuration; preserved for comparison.",
)
KNEE_GAIT_CONFIG = ANKLE_DERIVED_GAIT_CONFIG  # Compatibility: never silently retune old callers.


class GaitCurrentSource:
    """Adapter: degree-domain PID at configured rate -> current loop -> layer amps.

    `DrivetrainBenchSimulation` calls `current_source(k, theta_j, theta_j_dot)` once per
    PHYSICS step and hands the result straight to `DrivetrainLayer.advance` as rotor
    amps.  So this object owns three responsibilities and nothing else:

      1. run the PD law only every `steps_per_update` physics steps, HOLDING its output
         in between -- the source schedule determines the controller rate;
      2. advance `CurrentLoopModel` EVERY physics step, so a lagged current keeps
         responding between controller updates;
      3. rescale the modelled ACTUAL current into the layer's rotor-amp convention
         through `ConversionAudit`, so n_a is applied exactly once.

    It computes no torque and no belt force.  `last` exposes every intermediate signal
    for logging, with names that say which frame each one is in.
    """

    def __init__(self, cfg: JointCurrentConfig, audit: ConversionAudit, ref_rad,
                 steps_per_update: int):
        if int(steps_per_update) < 1:
            raise ValueError("steps_per_update must be >= 1")
        self.cfg = cfg
        self.audit = audit
        self.ref = ref_rad
        self.steps_per_update = int(steps_per_update)
        self.pid = CurrentPID(cfg.gains, cfg.limits, derivative_on=cfg.derivative_on,
                              integral_limit_deg_s=cfg.integral_limit_deg_s)
        self.loop = CurrentLoopModel(cfg.current_loop_tau_s)
        self.dt = None                           # set on the first call, from the model
        self.reset()

    def reset(self) -> None:
        self.pid.reset()
        self.loop.reset()
        self.held_cmd_A = 0.0
        self.updates = 0
        self.last = dict(ref_deg=0.0, meas_deg=0.0, error_deg=0.0,
                         pid_output_pre_sign_A=0.0, current_request_A=0.0,
                         current_applied_A=0.0, current_actual_modelled_A=0.0,
                         layer_rotor_current_A=0.0, saturated=0,
                         derivative_used=0, control_update=-1,
                         p_A=0.0, i_A=0.0, d_A=0.0)

    def __call__(self, k: int, theta_j: float, theta_j_dot: float) -> float:
        cfg = self.cfg
        dt = self.dt
        if dt is None:
            raise RuntimeError("GaitCurrentSource.dt was never set; the experiment must "
                               "set it from the compiled model's timestep before running")
        idx = k if k >= 0 else 0
        # ---- 1. the outer loop, at the selected rate, holding in between -----------
        if idx % self.steps_per_update == 0:
            ref_deg = cfg.to_controller_deg(float(self.ref[min(idx, len(self.ref) - 1)]))
            meas_deg = cfg.to_controller_deg(theta_j)
            self.held_cmd_A = self.pid.update(ref_deg, meas_deg,
                                              self.steps_per_update * dt)
            snap = self.pid.last
            self.updates += 1
            self.last.update(ref_deg=ref_deg, meas_deg=meas_deg,
                             error_deg=snap["error_deg"],
                             pid_output_pre_sign_A=snap["raw_A"],
                             current_request_A=snap["raw_signed_A"],
                             current_applied_A=self.held_cmd_A,
                             saturated=int(snap["saturated"]),
                             derivative_used=int(snap["derivative_used"]),
                             control_update=self.updates - 1,
                             p_A=snap["p_A"], i_A=snap["i_A"], d_A=snap["d_A"])
        # ---- 2. the current loop, EVERY physics step -------------------------------
        i_actual = self.loop.update(self.held_cmd_A, dt)
        # ---- 3. into the layer's rotor-amp convention, n_a applied once -------------
        i_layer = self.audit.layer_current_for(i_actual)
        self.last["current_actual_modelled_A"] = i_actual
        self.last["layer_rotor_current_A"] = i_layer
        return i_layer
