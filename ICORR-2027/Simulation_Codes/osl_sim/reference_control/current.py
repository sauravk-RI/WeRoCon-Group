#!/usr/bin/env python3
"""
ankle_current_control.py -- the PHYSICAL ankle controller's architecture, in simulation.

WHAT THIS MODULE IS FOR
    The original bench (`oslbench/control/servo.py` + `oslbench/sim/servo.py`) drives a
    MuJoCo POSITION actuator with a joint-torque-domain PD law in radians.  The physical
    ankle rig does something structurally different: an outer Python loop reads an
    external encoder in DEGREES, computes a CURRENT in amps, and hands that to the
    driver's current loop; the motor then drives the joint through a 9:1 planetary and
    an external belt.  Those are not the same control architecture and their gains are
    not comparable -- 600 N.m/rad and 0.75 A/deg are different kinds of number.

    This module implements the PHYSICAL architecture so the two can be compared without
    pretending they are the same thing.  It is additive: `controller.py`,
    `drivetrain.py` and `drivetrain_sim.py` are untouched, and the knee experiments and
    the oracle are unaffected.

WHAT IT REUSES AND WHAT IT ADDS
    REUSED, not reimplemented:
      * `oslbench.control.drivetrain.DrivetrainParameters` and its documented frame conventions
      * `oslbench.sim.compliant.DrivetrainLayer` -- Best et al.'s actuator-shaft and
        belt equations, verbatim.  The belt equations are NOT copied into this file.
      * `oslbench.control.servo.PDController(0, 0).write_to_model` -- the existing
        servo-disconnect trick, pointed at the ANKLE actuator instead of the knee.
    ADDED, because no equivalent exists:
      * the encoder convention boundary (degrees <-> radians, and the sign)
      * the degree-domain current PID the hardware script runs
      * ankle wiring: `DrivetrainBenchSimulation` is hard-wired to the knee dof, so the
        ankle needs its own small wiring class.  It composes the SAME layer.

THE THING THIS MODULE EXISTS TO MAKE EXPLICIT
    There are three places a factor can go missing or be applied twice: the 9:1
    planetary, the external belt, and the degree/radian boundary.  Every conversion here
    is a named object with its reference frame in its name, and
    `ConversionAudit.explain()` prints the whole chain with the frame at each stage so a
    reader can check that each ratio appears exactly once.

    NOT VALIDATED AGAINST HARDWARE.  Every actuator and belt parameter is paper-derived
    or reported-from-a-script; see docs/PHYSICAL_ANKLE_CONTROL_ALIGNMENT.md.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .drivetrain import PAPER, DrivetrainParameters

__all__ = [
    "CAD_BELT_TEETH", "CAD_BELT_RATIO", "ANKLE_PAPER_CAD_BELT",
    "SCRIPT_KT_OUTPUT_NM_PER_A", "MANUAL_ROTOR_KT_NM_PER_A",
    "TORQUE_CONSTANT_CHOICES", "torque_constant",
    "CurrentLimits", "EncoderConvention", "CurrentPID", "PIDGains",
    "ConversionAudit", "CurrentLoopModel",
    "check_sign_consistency",
]

# =====================================================================================
# RATIOS.  Each is named for the two frames it maps BETWEEN.
# =====================================================================================
# CONFIRMED from the CAD: the mesh assets in models/osl_v2_bench.xml are
# p_b0001_InputPulley_5mm_11teeth and p_b0012_OutputPulley_5mm_50teeth, and for the
# ankle the 50-tooth sits in body `foot` (joint side) while the 11-tooth sits in body
# `shank` (motor side).  So the ankle belt REDUCES by 50/11.
CAD_BELT_TEETH = (50, 11)                       # (output/joint side, input/motor side)
CAD_BELT_RATIO = CAD_BELT_TEETH[0] / CAD_BELT_TEETH[1]      # 4.545454...

# The paper's belt ratio is 4.61, which is NOT the CAD pair.  Both are kept, separately.
# `ANKLE_PAPER_CAD_BELT` is Best et al.'s actuator with the CAD's measured belt ratio --
# a deliberate hybrid, and labelled as one.  PAPER itself is never modified.
ANKLE_PAPER_CAD_BELT = PAPER.with_belt_ratio(CAD_BELT_RATIO)

# =====================================================================================
# THE TORQUE CONSTANT.  THREE MUTUALLY INCONSISTENT CANDIDATES.  READ THIS.
# =====================================================================================
# REPORTED from the hardware script (`KT = 0.5701  # N*m/A, output-shaft torque
# constant`).  NOT verified here: that script is not in this workspace, so every number
# attributed to it in this module is second-hand.
SCRIPT_KT_OUTPUT_NM_PER_A = 0.5701

# REPORTED as an AK80-9 V3.0 figure, shaft reference NOT stated in the source we have.
MANUAL_ROTOR_KT_NM_PER_A = 0.095

# The three candidates, all referred to the ACTUATOR OUTPUT shaft so they are
# comparable, and all PROVISIONAL:
#
#   script         0.5701 N.m/A   as the script labels it (output side)
#   paper          0.9972 N.m/A   = PAPER.k_t (rotor, 0.1108) * n_a (9)
#   manual_rotor   0.8550 N.m/A   = 0.095 (taken as rotor) * n_a (9)
#
# They disagree by up to 1.75x, which is far too much to pick by eye.  One numerical
# coincidence is worth recording because it is a checkable hypothesis rather than a
# guess:  0.5701 / 0.095 = 6.0011.  That is the AK80-**6** reduction, not the AK80-**9**
# the hardware reportedly uses.  If the script's KT was built by applying 6:1 to a
# ~0.095 N.m/A rotor constant, it under-states output torque by 9/6 = 1.5x -- and every
# torque the script logged, including the informally eyeballed 4.8-5 N.m peak, would be
# low by that factor.  This is a HYPOTHESIS from a 4-significant-figure coincidence.  It
# is not established, it must not be assumed, and resolving it needs the manufacturer's
# specification for the exact actuator variant.  Until then the choice is a CLI flag and
# the default reproduces the script.
TORQUE_CONSTANT_CHOICES = {
    "script": (SCRIPT_KT_OUTPUT_NM_PER_A,
               "REPORTED from the hardware script as an output-shaft constant; "
               "provenance UNRESOLVED (equals 0.095 x 6, the AK80-6 ratio)"),
    "paper": (PAPER.k_t * PAPER.n_a,
              "DERIVED: PAPER rotor k_t x n_a. Internally consistent with the "
              "Option-C drivetrain model, but a different actuator build"),
    "manual_rotor": (MANUAL_ROTOR_KT_NM_PER_A * PAPER.n_a,
                     "DERIVED: the ~0.095 N.m/A figure READ AS A ROTOR constant, "
                     "x n_a = 9. Shaft reference of the source value is unconfirmed"),
}


def torque_constant(choice: str = "script") -> tuple[float, str]:
    """The ACTUATOR-OUTPUT torque constant for a named parameterisation, N.m/A.

    Returns (value, provenance).  There is no default that is known to be correct; the
    default merely reproduces what the hardware script did.
    """
    if choice not in TORQUE_CONSTANT_CHOICES:
        raise ValueError(f"torque-constant choice must be one of "
                         f"{sorted(TORQUE_CONSTANT_CHOICES)}, got {choice!r}")
    return TORQUE_CONSTANT_CHOICES[choice]


# =====================================================================================
# CURRENT LIMITS.  A command-range ceiling is not a safe operating limit.
# =====================================================================================
@dataclass(frozen=True)
class CurrentLimits:
    """Three different numbers that are routinely confused.

    `sim_clip_A` is the only one this simulation enforces.  It defaults to the value the
    hardware script reportedly uses so that the simulated controller saturates where the
    physical one does -- which is the point of the comparison.

    IT IS NOT A STATEMENT THAT 60 A IS SAFE.  60 A is a software ceiling in a script; the
    reported AK80-9 V3.0 figures are ~12 A rated and ~28 A peak, and we could not verify
    which of those is a motor rating, a driver-board rating, or a protocol command range.
    Whether any current is safe to command on the physical rig requires the lab
    supervisor and the manufacturer's documentation. Nothing here authorises a hardware
    test.
    """

    sim_clip_A: float = 60.0
    reported_rated_A: float | None = 12.0
    reported_peak_A: float | None = 28.0
    provenance: str = ("sim_clip_A REPORTED from the hardware script's code-level clip; "
                       "rated/peak REPORTED for AK80-9 V3.0 with the motor-vs-driver "
                       "distinction UNCONFIRMED. Simulation-only.")

    def classify(self, amps: float) -> str:
        """Which reported envelope a current falls in.  Labels, not permissions."""
        a = abs(float(amps))
        if self.reported_rated_A is not None and a <= self.reported_rated_A:
            return "within reported rated"
        if self.reported_peak_A is not None and a <= self.reported_peak_A:
            return "above reported rated, within reported peak"
        return "ABOVE reported peak -- simulation only, not a permitted command"


# =====================================================================================
# THE ENCODER BOUNDARY.  Degrees and a sign live here and nowhere else.
# =====================================================================================
@dataclass(frozen=True)
class EncoderConvention:
    """MAPPING 1: `mjcf_q_to_encoder_sign`.  MuJoCo ankle coordinate -> encoder degrees.

        encoder_deg = mjcf_q_to_encoder_sign * degrees(q_ankle_rad) + offset_deg

    THIS IS NOT THE SAME QUANTITY AS THE CONTROLLER'S MOTOR SIGN, AND MUST NOT BE
    DERIVED FROM IT.  The hardware script's `ENCODER_TO_MOTOR_SIGN = -1` is MAPPING 2
    (`controller_output_to_motor_current_sign`, carried in `PIDGains`): it maps the
    controller's output to the current command, inside the controller.  Mapping 1 is a
    statement about COORDINATE FRAMES -- which way the encoder counts relative to the
    model's generalized coordinate.  Nothing about mapping 2 constrains it.

    I previously claimed "three independent lines" supported sign = -1.  That was wrong:
    two of the three (the loop-stability condition, and "the physical test ran without
    running away") both involve mapping 2, so they were circular.  Only the ROM-asymmetry
    argument was independent, and it is weak on its own.  The derivation below replaces
    all of it.

    ===================================================================================
    MAPPING 1, DERIVED FROM GEOMETRY.  No reference to mapping 2 anywhere.
    ===================================================================================

    Inputs, and where each comes from:

      (a) ankle hinge axis = (0, 1, 0)                    CONFIRMED from the MJCF.
          (The knee's is (0, -1, 0); the two joints do not share a convention, so the
          ankle's cannot be assumed from the knee's.)
      (b) the foot geometry, measured from the Ossur Low Profile Vari-Flex mesh
          (`z_lowProfileVariFlexFoot_ossur_VLPE5XX0.stl`), transformed by its geom
          pos/quat into the foot-body frame whose origin IS the ankle joint:
              x in [-0.1665, +0.0756] m,  z in [-0.0915, -0.0350] m
              extent 242 mm along X vs 56 mm along Z
          CONFIRMED: parsed from the STL vertices, not from the mesh-frame origin.
      (c) ANTERIOR = -X.  This is now SELF-ESTABLISHING from (b) rather than taken from
          the project's CAD notes: the foot's long axis is X, and it extends 167 mm one
          way against 76 mm the other.  On a prosthetic foot the long side is the
          forefoot and the short side the heel, so the 167 mm direction (-X) is anterior.
      (d) plantarflexion = the toe moves DOWN (and the heel up).  Definitional.

    Derivation.  For a hinge about +Y, a positive rotation sends
    z' = -x sin(theta) + z cos(theta), so dz/dtheta|0 = -x:

        toe   at x = -0.1665  ->  dz/dtheta = +0.1665 m/rad  ->  positive theta RAISES it
        heel  at x = +0.0756  ->  dz/dtheta = -0.0756 m/rad  ->  positive theta LOWERS it

    Toe up together with heel down is DORSIFLEXION, and both ends agree, so the answer
    does not depend on which point is chosen.  Checked at finite angles too (+20 deg
    lifts the toe 61.6 mm and drops the heel 20.9 mm), so it is not a small-angle
    artefact.

        => positive MuJoCo ankle q = DORSIFLEXION
        => the encoder calls PLANTARFLEXION positive (REPORTED)
        => the two conventions OPPOSE:  mjcf_q_to_encoder_sign = -1

    CORROBORATION, also independent of mapping 2: under this mapping the authored ROM
    [-30, +20] deg reads as 30 deg plantarflexion and 20 deg dorsiflexion.  A prosthetic
    ankle allows more plantarflexion than dorsiflexion, so the asymmetry agrees.

    ===================================================================================
    THE ONE RESIDUAL UNCERTAINTY, and it is a premise, not a step
    ===================================================================================
    Input (d) pairs with the REPORTED claim that the external encoder calls
    plantarflexion positive.  The hardware script is not in this workspace, so that
    claim is second-hand.  The geometry above is solid; the mapping inherits exactly the
    confidence of that one premise.  **If the encoder actually counts dorsiflexion
    positive, mapping 1 flips to +1** and nothing else in the derivation changes.

    So this is PROVISIONAL-BY-PREMISE, not unresolved-by-geometry.  It stays an explicit
    parameter (`--encoder-sign`), the run prints which way it was obtained, and the
    positive-feedback detector remains armed because a wrong mapping does not crash.
    """

    sign: int = -1
    offset_deg: float = 0.0
    positive_means: str = "plantarflexion"
    provenance: str = ("mjcf_q_to_encoder_sign = -1, DERIVED from the ankle axis (0,1,0) "
                       "and the Vari-Flex foot STL extents (toe at x=-0.1665 m, heel at "
                       "x=+0.0756 m): positive q raises the toe = dorsiflexion, which "
                       "opposes the encoder's reported positive-plantarflexion. "
                       "PROVISIONAL only in that the encoder's polarity is REPORTED, "
                       "not verified. Independent of the controller's motor sign.")

    def to_encoder_deg(self, q_rad: float) -> float:
        """MuJoCo ankle coordinate (rad) -> encoder reading (deg)."""
        return self.sign * math.degrees(float(q_rad)) + self.offset_deg

    def to_encoder_deg_per_s(self, qdot_rad_s: float) -> float:
        """MuJoCo ankle velocity (rad/s) -> encoder-frame rate (deg/s).  No offset."""
        return self.sign * math.degrees(float(qdot_rad_s))

    def to_model_rad(self, enc_deg: float) -> float:
        """Encoder reading (deg) -> MuJoCo ankle coordinate (rad).  The exact inverse."""
        return math.radians((float(enc_deg) - self.offset_deg) / self.sign)

    def rom_in_encoder_frame(self, ankle_range_rad) -> tuple[float, float]:
        """The authored ROM in ENCODER degrees.  The interval ENDS SWAP for sign < 0."""
        a, b = (self.to_encoder_deg(x) for x in ankle_range_rad)
        return (min(a, b), max(a, b))




# Kept as an alias so nothing that imported the old name breaks, but the new name says
# WHICH mapping it is -- the whole point of this pass.



def check_sign_consistency(enc, gains, params) -> dict:
    """Trace both mappings through the torque chain and report, without asserting.

    Deliberately NOT a derivation of either mapping: it takes both as given and asks
    whether the closed loop is negative feedback. That is a CONSEQUENCE of the two, and
    reporting it as such is what keeps it from being used to infer mapping 1 (which is
    the circularity this pass exists to remove).

    The chain, all of it already established elsewhere:
        meas    = s_enc * deg(q)                        mapping 1
        err     = ref - meas
        I_cmd   = s_mot * (Kp*err + ...)                mapping 2
        I_q > 0 -> tau_m > 0 -> theta_a rises -> theta_s = theta_j - theta_a/n_t < 0
                -> paper (4) gives tau_j > 0 -> qfrc_applied > 0 -> q rises
        d(meas) = s_enc * dq
        => sign(d_meas) = s_enc * s_mot * sign(err), so negative feedback needs
           s_enc * s_mot = +1.
    """
    s_enc, s_mot = int(enc.sign), int(gains.encoder_to_motor_sign)
    product = s_enc * s_mot
    return dict(
        mjcf_q_to_encoder_sign=s_enc,
        controller_output_to_motor_current_sign=s_mot,
        product=product,
        negative_feedback=bool(product > 0),
        torque_chain_positive_current_raises_q=True,
        note=("the two mappings agree (product +1), so the loop is negative feedback"
              if product > 0 else
              "the two mappings OPPOSE (product -1), so the loop is POSITIVE feedback: "
              "it will drive away from the reference and park against a joint stop "
              "WITHOUT going non-finite"),
    )


# =====================================================================================
# THE HARDWARE CONTROL LAW.  Degree domain in, amps out.
# =====================================================================================
@dataclass(frozen=True)
class PIDGains:
    """The physical script's reported baseline gains.  NOT tuned, NOT validated.

    Units matter and are not the bench's: Kp is A/deg and Kd is A/(deg/s), so these
    numbers are not comparable with the bench's 600 N.m/rad and 17.253 N.m.s/rad.
    """

    kp_A_per_deg: float = 1.5
    ki_A_per_deg_s: float = 0.0
    kd_A_per_deg_per_s: float = 0.03
    # MAPPING 2: `controller_output_to_motor_current_sign`.  The script's
    # ENCODER_TO_MOTOR_SIGN. It maps the controller's output to the current command and
    # lives entirely INSIDE the controller. It is NOT a coordinate-frame statement and
    # must never be used to infer mapping 1 (`EncoderConvention.sign`) -- that was the
    # circularity this field's name is now explicit about.
    encoder_to_motor_sign: int = -1
    provenance: str = ("REPORTED from the hardware script; that file is NOT in this "
                       "workspace, so these values are second-hand and unverified")

    @property
    def controller_output_to_motor_current_sign(self) -> int:
        """Mapping 2, under the name this project uses for it in writing."""
        return int(self.encoder_to_motor_sign)


class CurrentPID:
    """The outer loop, reproduced as the physical script reportedly behaves.

        e_deg  = ref_deg - meas_deg
        I_raw  = Kp*e_deg + Ki*integral(e_deg dt) + Kd*de_deg/dt
        I_cmd  = sign * clip(I_raw, -clip_A, +clip_A)

    FIVE BEHAVIOURS COPIED DELIBERATELY, because changing any of them silently would
    make the comparison meaningless:

    1. DERIVATIVE ON ERROR, not on measurement.  With a step reference these differ by
       an impulse at the step, which is exactly where a step response is read.  A
       derivative-on-measurement variant is available as an explicit flag and is NOT the
       default.
    2. FIRST-UPDATE DERIVATIVE SUPPRESSED.  The script's `prev_error_deg` is unset on
       the first call, so its derivative term contributes nothing on that update.  With
       Kd = 0.03 A/(deg/s) and a 5 deg step at 1 kHz, an unsuppressed first derivative
       would be 0.03 * 5/0.001 = 150 A -- i.e. the difference between saturating
       immediately and not saturating at all.  This is the single most consequential
       detail in the file.
    3. Ki DEFAULTS TO ZERO, so the law is a PD controller.  The integrator is
       implemented anyway, with clamping anti-windup, so a future nonzero Ki is
       testable; with Ki = 0 the integral state must stay exactly 0.0.
    4. THE SIGN IS APPLIED AFTER SATURATION, so a symmetric clip stays symmetric.
    5. CLIPPING IS A HARD CLAMP on the raw command, and saturation is reported.

    The controller holds no model and no timestep of its own: `dt` is passed in per
    update, so the scheduler owns the rate.
    """

    def __init__(self, gains: PIDGains = PIDGains(), limits: CurrentLimits = CurrentLimits(),
                 derivative_on: str = "error", integral_limit_deg_s: float | None = None):
        if derivative_on not in ("error", "measurement"):
            raise ValueError("derivative_on must be 'error' or 'measurement'")
        self.gains = gains
        self.limits = limits
        self.derivative_on = derivative_on
        if integral_limit_deg_s is not None and (not math.isfinite(integral_limit_deg_s) or integral_limit_deg_s < 0):
            raise ValueError('Integral limit must be finite and nonnegative')
        self.integral_limit_deg_s = integral_limit_deg_s
        self.reset()

    def reset(self) -> None:
        """Back to the state the script is in when PID engages."""
        self.integral_deg_s = 0.0
        self.prev_error_deg: float | None = None        # None == "unset", see note 2
        self.prev_meas_deg: float | None = None
        self.updates = 0
        self.last = dict(error_deg=0.0, p_A=0.0, i_A=0.0, d_A=0.0,
                         raw_A=0.0, raw_signed_A=0.0, cmd_A=0.0,
                         saturated=False, derivative_used=False)

    def update(self, ref_deg: float, meas_deg: float, dt: float) -> float:
        """One controller update.  Returns the commanded current in amps."""
        if not (dt > 0.0):
            raise ValueError(f"dt must be > 0, got {dt!r}")
        g = self.gains
        err = float(ref_deg) - float(meas_deg)

        p = g.kp_A_per_deg * err

        # Integral with clamping anti-windup, expressed in the pre-sign raw domain so
        # the clamp and the output clip agree. With Ki = 0 this stays exactly 0.0.
        if g.ki_A_per_deg_s != 0.0:
            self.integral_deg_s += err * float(dt)
            # None preserves the historical limit; knee tuning supplies 50 deg.s.
            lim = self.limits.sim_clip_A / abs(g.ki_A_per_deg_s)
            if self.integral_limit_deg_s is not None:
                lim = self.integral_limit_deg_s
            self.integral_deg_s = max(-lim, min(lim, self.integral_deg_s))
        i = g.ki_A_per_deg_s * self.integral_deg_s

        # Derivative.  Suppressed on the first update, as the script's unset
        # `prev_error_deg` suppresses it.
        used = False
        d = 0.0
        if self.derivative_on == "error":
            if self.prev_error_deg is not None:
                d = g.kd_A_per_deg_per_s * (err - self.prev_error_deg) / float(dt)
                used = True
        else:
            if self.prev_meas_deg is not None:
                d = -g.kd_A_per_deg_per_s * (float(meas_deg) - self.prev_meas_deg) / float(dt)
                used = True

        raw = p + i + d
        clip = float(self.limits.sim_clip_A)
        clipped = max(-clip, min(clip, raw))
        saturated = clipped != raw
        cmd = g.encoder_to_motor_sign * clipped

        self.prev_error_deg = err
        self.prev_meas_deg = float(meas_deg)
        self.updates += 1
        self.last = dict(
            error_deg=err, p_A=p, i_A=i, d_A=d,
            raw_A=raw,                                  # pre-sign, pre-clip
            # The SAME frame as `cmd_A`, so a log can compare request against applied
            # and see ONLY the effect of clipping. Comparing `raw_A` with `cmd_A`
            # instead would show the motor sign on every step and hide saturation,
            # which is the opposite of what a request/applied pair is for.
            raw_signed_A=g.encoder_to_motor_sign * raw,
            cmd_A=cmd, saturated=bool(saturated), derivative_used=used)
        return cmd


# =====================================================================================
# THE CONVERSION AUDIT.  Where every ratio is applied, exactly once.
# =====================================================================================
@dataclass(frozen=True)
class ConversionAudit:
    """The current -> torque -> joint chain, with the frame named at every stage.

    Built so the chain can be PRINTED and checked by a reader rather than trusted.
    Two conventions are deliberately incompatible and must not be mixed:

      PAPER/Option-C chain, used by `DrivetrainLayer`:
          I_q (rotor amps) --x k_t--> rotor N.m --x n_a--> actuator-output N.m
          and the belt is a COMPLIANT element, so joint torque comes from the belt
          deflection, not from a ratio: tau_j = -sgn(th_s)(p1|th_s| + p2|th_s|^2).
          `k_t_joint = k_t * n_a * n_t` is the RIGID, QUASI-STATIC slope only.

      SCRIPT chain, as the hardware logged it:
          I_q --x KT--> "output-shaft" N.m --x n_belt--> an EQUIVALENT joint torque.
          That second multiply is a RIGID assumption. It is a scale, not a measurement.

    `k_t_output` below is the ONE number that decides which chain is in use, and it is
    chosen explicitly.
    """

    k_t_output_Nm_per_A: float
    k_t_output_choice: str
    k_t_output_provenance: str
    belt_ratio: float
    belt_ratio_source: str
    params: DrivetrainParameters
    limits: CurrentLimits = field(default_factory=CurrentLimits)

    @property
    def rotor_kt_implied(self) -> float:
        """k_t_output / n_a -- the rotor constant the chosen output constant implies."""
        return self.k_t_output_Nm_per_A / self.params.n_a

    @property
    def rigid_joint_kt(self) -> float:
        """k_t_output * belt_ratio [N.m/A] -- RIGID, QUASI-STATIC joint-side slope.

        Not used to drive the simulation: the belt in `DrivetrainLayer` is compliant and
        produces the joint torque from its deflection. This exists so the script's
        `tau * n_belt` column can be reproduced and compared, and so a reader can see
        that the belt ratio is applied exactly ONCE on that path.
        """
        return self.k_t_output_Nm_per_A * self.belt_ratio

    def layer_current_for(self, i_q_output_frame: float) -> float:
        """Convert a current meant for `k_t_output` into the ROTOR amps the layer wants.

        THIS IS THE DOUBLE-COUNT GUARD, and it is the most important method here.
        `DrivetrainLayer` implements paper eq (1) as tau_m = I_q * k_t * n_a with the
        PAPER rotor k_t, so feeding it a current sized for a different output constant
        would silently apply the wrong torque per amp. The current is therefore rescaled
        by the ratio of output constants -- which leaves the amps' MEANING unchanged
        (same physical torque at the output) while letting the layer keep its own,
        already-correct, internal gear bookkeeping.

        Equivalent to: desired output torque / the layer's own output constant.
        """
        return float(i_q_output_frame) * (self.k_t_output_Nm_per_A
                                          / self.params.k_t_actuator)

    def explain(self) -> list[str]:
        """The whole chain as text, one stage per line, with the frame named."""
        p = self.params
        return [
            f"CURRENT -> TORQUE -> JOINT, with the frame at every stage",
            f"  chosen output constant   k_t_output = {self.k_t_output_Nm_per_A:.6f} "
            f"N.m/A  [ACTUATOR OUTPUT]   choice={self.k_t_output_choice!r}",
            f"    provenance             {self.k_t_output_provenance}",
            f"  implied rotor constant   {self.rotor_kt_implied:.6f} N.m/A  [ROTOR] "
            f"(= k_t_output / n_a, n_a = {p.n_a:g})",
            f"  PAPER rotor constant     {p.k_t:.6f} N.m/A  [ROTOR]",
            f"  PAPER output constant    {p.k_t_actuator:.6f} N.m/A  [ACTUATOR OUTPUT] "
            f"(= k_t * n_a)",
            f"  rescale applied to the layer's current: "
            f"x{self.k_t_output_Nm_per_A / p.k_t_actuator:.6f}  "
            f"(so n_a is applied ONCE, inside the layer)",
            f"  belt ratio               n_t = {self.belt_ratio:.6f}  "
            f"[ACTUATOR OUTPUT -> JOINT]   source={self.belt_ratio_source}",
            f"  joint torque in the sim  from the COMPLIANT belt, "
            f"tau_j = -sgn(th_s)(p1|th_s| + p2|th_s|^2); the ratio enters only through "
            f"th_s = theta_j - theta_a/n_t, i.e. ONCE, in the kinematics",
            f"  rigid-equivalent slope   {self.rigid_joint_kt:.6f} N.m/A  [JOINT] "
            f"(= k_t_output * n_t)  QUASI-STATIC SCALE ONLY, not how the sim works",
            f"  current clip             {self.limits.sim_clip_A:g} A (simulation only); "
            f"reported rated {self.limits.reported_rated_A} A, peak "
            f"{self.limits.reported_peak_A} A -- NOT a safety authorisation",
        ]


# =====================================================================================
# THE DRIVER'S CURRENT LOOP.  OPTIONAL, OFF BY DEFAULT, AND EXPLORATORY WHEN ON.
# =====================================================================================
class CurrentLoopModel:
    """First-order lag between the commanded current and the current the motor sees.

    WHAT THIS IS AND IS NOT
        The physical driver closes an inner current loop in its own firmware at a rate
        and bandwidth we have never measured.  Up to now this project ASSUMED that loop
        was ideal: the applied current equalled the clipped command within one control
        interval.  This class makes that assumption adjustable so its CONSEQUENCES can
        be measured, which is a different thing from knowing the driver's dynamics.

        **A nonzero tau_i here is an illustrative model assumption, not a measured
        hardware value.**  Nothing in this repository establishes the real driver's
        time constant.  Sweeping tau_i tells you how sensitive the simulated result is
        to that unknown; it cannot tell you what the unknown is.

    THE MODEL

        dI_actual/dt = (I_target - I_actual) / tau_i

    integrated EXACTLY over a step of length dt, rather than with a forward-Euler
    approximation:

        I_next = I_target + (I_previous - I_target) * exp(-dt / tau_i)

    The exact form matters at the ratios in play here: with tau_i = 1 ms and
    dt = 0.5 ms, forward Euler would use a coefficient of (1 - dt/tau) = 0.5 against the
    true exp(-0.5) = 0.6065, a 21 % error in the per-step decay, and it becomes unstable
    for dt > 2*tau.  The exponential form is unconditionally stable and exact for a
    piecewise-constant target, which is precisely what a held controller command is.

    tau_i = 0 IS A SPECIAL CASE, NOT A LIMIT
        It assigns the target directly.  It does NOT evaluate exp(-dt/0), and it
        introduces no one-step delay, so the ideal-loop path is reproduced BIT-FOR-BIT
        rather than approximately.  `tests/test_ankle_current_step.py` asserts that
        equality rather than trusting this paragraph.

    WHERE IT SITS IN THE CHAIN, AND WHAT IT MUST NOT DO
        It operates on AMPS ONLY, between the controller's clipped command and the
        drivetrain layer's input:

            PID -> sign -> clip -> [ THIS ] -> ConversionAudit rescale -> DrivetrainLayer

        It applies NO planetary ratio and NO belt ratio.  Those live in
        `ConversionAudit.layer_current_for` (n_a, once) and in the layer's own
        kinematics (n_t, once), and duplicating either here would be a silent 9x or
        4.5x torque error.  The state is a current in the CONTROLLER's amp convention,
        i.e. amps against the chosen `k_t_output`.

    INITIAL CONDITION
        `i0` amps, default 0.0: the rig is at rest with no commanded current when PID
        engages, so the loop starts from zero.  `reset()` returns to that same value,
        and it is recorded in the run metadata rather than left implicit.

    TIMING
        `update` is called once per PHYSICS step with that step's dt, while the outer
        controller runs at 1 kHz and holds its command across the two 0.5 ms steps.  So
        the current continues to evolve between controller updates, which is the whole
        point: a lagged loop does not wait for the next command to keep responding.
    """

    def __init__(self, tau_s: float = 0.0, i0: float = 0.0):
        tau = float(tau_s)
        if not math.isfinite(tau):
            raise ValueError(f"current-loop time constant must be finite, got {tau_s!r}")
        if tau < 0.0:
            raise ValueError(
                f"current-loop time constant must be >= 0 s, got {tau_s!r}. A negative "
                f"time constant would make exp(-dt/tau) grow without bound -- that is "
                f"not a slow current loop, it is a divergent one.")
        if not math.isfinite(float(i0)):
            raise ValueError(f"initial current must be finite, got {i0!r}")
        self.tau_s = tau
        self.i0 = float(i0)
        self.i_actual = float(i0)
        self.steps = 0

    @property
    def ideal(self) -> bool:
        """True when this is a passthrough, i.e. the historical ideal-loop path."""
        return self.tau_s == 0.0

    def reset(self) -> None:
        """Back to the documented initial condition."""
        self.i_actual = self.i0
        self.steps = 0

    def update(self, i_target_A: float, dt_s: float) -> float:
        """Advance the modelled actual current by one physics step.  Returns amps.

        `i_target_A` is the ALREADY signed and ALREADY clipped command.  This method
        neither signs nor clips: doing either here would apply it twice.
        """
        tgt = float(i_target_A)
        if not math.isfinite(tgt):
            raise ValueError(f"current target must be finite, got {i_target_A!r}")
        dt = float(dt_s)
        if not (dt > 0.0) or not math.isfinite(dt):
            raise ValueError(f"dt must be finite and > 0, got {dt_s!r}")
        self.steps += 1
        if self.tau_s == 0.0:
            self.i_actual = tgt                     # exact passthrough, no lag, no delay
            return self.i_actual
        decay = math.exp(-dt / self.tau_s)
        self.i_actual = tgt + (self.i_actual - tgt) * decay
        return self.i_actual

    def describe(self) -> str:
        if self.ideal:
            return ("ideal current loop (tau_i = 0): the applied current equals the "
                    "clipped command exactly. ASSUMPTION, not a measurement.")
        return (f"first-order current loop, tau_i = {self.tau_s * 1e3:.3f} ms, exact "
                f"exponential update, i0 = {self.i0:g} A. ILLUSTRATIVE MODEL "
                f"ASSUMPTION -- the real driver's time constant is unknown.")

    def __repr__(self) -> str:                                  # pragma: no cover
        return (f"CurrentLoopModel(tau_s={self.tau_s:g}, i0={self.i0:g}, "
                f"i_actual={self.i_actual:g})")


# =====================================================================================
# THE ANKLE WIRING.  Composes DrivetrainLayer; does not reimplement it.
# =====================================================================================
