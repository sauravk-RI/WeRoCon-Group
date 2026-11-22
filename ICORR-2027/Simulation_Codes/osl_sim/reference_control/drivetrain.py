"""
oslbench.control.drivetrain -- the OSL V2 actuator + belt-transmission model of Best et al. 2025,
as a standalone dynamics module.  NO MuJoCo, NO numpy, NO file I/O, NO plotting.

SOURCE.  Every equation and every number in this file comes from:

    T. K. Best, G. C. Thomas, S. R. Ayyappan, R. D. Gregg, E. J. Rouse,
    "A Compensated Open-Loop Impedance Controller Evaluated on the Second-Generation
    Open-Source Leg Prosthesis," IEEE/ASME Trans. Mechatronics, vol. 30, no. 6,
    pp. 4732-4743, Dec. 2025.  DOI 10.1109/TMECH.2024.3508469.
    Section III-A ("Drivetrain Model"), equations (1)-(5), and Fig. 3 / Fig. 4.

    The actuator model (1)-(2) is credited by Best et al. to Nesler et al. (their
    refs [27], [44]); the parameter VALUES in this file are the ones Best et al.
    regressed for the OSL V2 actuator, quoted in Section III-A1.

    Parameter provenance -- which number is measured, which is paper-derived, which is
    CAD-derived, which is assumed -- is tabulated in docs/DRIVETRAIN_PARAMETERS.md.
    That table, not this docstring, is the authority on provenance.

WHAT THIS MODULE IS FOR
    To encode those equations cleanly, make every parameter traceable, and be
    unit-testable, BEFORE any of it is integrated into the MuJoCo bench.  It is
    deliberately not wired into models/osl_v2_bench.xml, oslbench/sim/model.py,
    oslbench/sim/servo.py or oslbench/control/servo.py.  Nothing in the validated
    benchmark imports this file.

WHAT THIS MODULE IS NOT
    Not a controller (the paper's (6)-(21) compensator is NOT implemented here).
    Not a measurement of our hardware -- we have no OSL V2 actuator on a dynamometer.
    Every number here is the PAPER's, identified on the PAPER's units and frames.


================================================================================
COORDINATE AND TORQUE CONVENTIONS -- READ THIS BEFORE USING ANY QUANTITY
================================================================================

There are THREE shafts and it matters constantly which one a quantity lives on.

    rotor  --[ n_a = 9 planetary, inside the actuator ]--> ACTUATOR OUTPUT
    ACTUATOR OUTPUT = belt input pulley                     angle theta_a
           --[ n_t = 4.61 single-stage belt, COMPLIANT ]--> JOINT
    JOINT  = belt output pulley = knee/ankle axis            angle theta_j

  theta_a         rad      ANGLE OF THE ACTUATOR OUTPUT SHAFT.  This is the shaft
                           AFTER the 9:1 planetary and BEFORE the belt -- i.e. the
                           belt's input pulley.  It is NOT the rotor angle: the rotor
                           turns n_a = 9 times faster.
  theta_a_dot     rad/s    its velocity
  theta_a_ddot    rad/s^2  its acceleration

  theta_j         rad      ANGLE OF THE PROSTHETIC JOINT (knee or ankle axis), i.e.
                           the belt's output pulley.  This is what a joint encoder
                           reads and what a MuJoCo hinge coordinate would be.

  theta_s         rad      BELT DEFLECTION, REFERRED TO THE JOINT SIDE.
                           Defined by the paper's (3):   theta_j = theta_a/n_t + theta_s
                           so equivalently               theta_s = theta_j - theta_a/n_t
                           theta_s is joint-side because it is added to theta_a/n_t,
                           which is already a joint-side angle.  A deflection of
                           theta_s rad is theta_s rad seen at the JOINT, not at the
                           actuator.  In a rigid drivetrain theta_s == 0 identically.

WHICH SIDE EACH PARAMETER LIVES ON
  AT THE ACTUATOR OUTPUT SHAFT (theta_a):   J_a, B_a, f_c, f_g, tau_m, tau_a
  AT THE JOINT (theta_j):                   p1, p2, K_s, rho, tau_s, tau_j
  NEITHER (electrical/rotor):               k_t, I_q      (k_t is the ROTOR's
                                            torque constant; k_t*n_a is the
                                            actuator-output torque constant)

J_a IS NOT "JOINT INERTIA" AND IS NOT "ROTOR INERTIA"
    The paper's words for J_a are: "J_a captures the combined effects of rotor and
    gearbox inertial torques due to the actuator's acceleration theta_a_ddot."  It is
    measured at the ACTUATOR OUTPUT, and the n_a^2 reflection of the rotor is ALREADY
    INSIDE IT.  Two consequences that are easy to get wrong:

      * Do NOT multiply J_a by n_a^2.  That would double-count the planetary.
      * J_a is NOT the joint-side inertia.  Reflecting it to the joint costs a
        further n_t^2 -- see `reflect_actuator_inertia_to_joint`, and read the
        warning in its docstring before using the result for anything.

    For reference only, the rotor inertia IMPLIED by J_a is J_a/n_a^2 = 1.21e-4
    kg.m^2.  The paper does not report a rotor inertia separately, so that number is
    DERIVED, not measured.

TORQUES
  tau_m   N.m   at theta_a.  Motor output torque, paper (1): tau_m = I_q * k_t * n_a.
                The name is the paper's; note it is expressed at the actuator OUTPUT,
                not at the rotor.  Rotor torque would be I_q * k_t.
  tau_f   N.m   at theta_a.  Friction loss, paper (2), OPPOSES motion.
  tau_a   N.m   at theta_a.  ACTUATOR OUTPUT TORQUE -- what the actuator delivers into
                the belt after its own inertia, damping and friction have been paid.
  tau_s   N.m   at theta_j.  "the torque in the transmission" (paper, Sec. III-A2);
                the quantity plotted as "Belt Torque" in the paper's Fig. 3.
  tau_j   N.m   at theta_j.  JOINT TORQUE: the torque the drivetrain applies TO the
                joint/foot.  Paper (4): tau_j = -tau_s = -sgn(theta_s)*rho(|theta_s|).

THE JOINT TORQUE IS *NOT* SIMPLY (TOTAL RATIO) x (MOTOR TORQUE)
    It is tempting to write tau_j = n_a * n_t * I_q * k_t.  That is the RIGID,
    frictionless, quasi-static, zero-acceleration limit, and it is wrong in general
    here, because the belt is compliant.  The paper gives TWO expressions for tau_j
    and it is worth being precise about why both are true at once:

      (A) FROM THE BELT STATE, paper (4):
              tau_j = -sgn(theta_s) * rho(|theta_s|)
          The transmitted torque is set by how far the belt is stretched.  Nothing
          about the motor appears.  This is a spring law: deflect the joint in +
          and the drivetrain pushes back in -.

      (B) FROM THE ACTUATOR SIDE, paper (7) and its surrounding text
          ("The actuator torque and the joint torque are related by tau_j = n_t tau_a"):
              tau_j = n_t * tau_a
          The belt itself is modelled as MASSLESS, so torque in = torque out, scaled
          by the ratio.  n_a does not appear because it is already inside tau_a.

    (A) and (B) are the same torque viewed from either side of the spring, and
    together they are the constraint that closes the system: the belt deflection
    determines the load the actuator must react against,

              tau_a = tau_j / n_t = -sgn(theta_s) * rho(|theta_s|) / n_t

    and that tau_a, substituted into the rearranged (1), gives the actuator's
    acceleration:

              J_a * theta_a_ddot = tau_m - tau_f - B_a*theta_a_dot - tau_a

    Substituting (A) for tau_a shows where the total ratio really does appear.  In
    steady state (theta_a_ddot = 0, theta_a_dot = 0, so tau_f is not yet mobilised and
    the damping term vanishes) we get tau_a = tau_m and therefore

              tau_j = n_t * tau_m = n_t * n_a * k_t * I_q

    -- the naive formula, but ONLY in that limit.  Away from it, the difference is
    exactly the actuator's inertial, viscous and frictional torques multiplied by n_t,
    which is the entire point of the paper's compensator.

TOTAL RATIO -- AND A NUMBER IT MUST NEVER BE CONFUSED WITH
    n_a           = 9         planetary reduction inside the actuator     (paper)
    n_t           = 4.61      single-stage belt reduction                 (paper)
    n_a * n_t     = 41.49     TOTAL NOMINAL MECHANICAL RATIO of OSL V2    (paper)

    Our own CAD export of the OSL V2 belt gives a 50/11 pulley-tooth ratio =
    4.5455, hence a total of 40.909.  The two belt ratios differ by 1.4 %.  Both are
    recorded; NEITHER is silently substituted for the other.  `PAPER` below uses the
    paper's 4.61; `CAD_BELT` holds 50/11 for comparison only.  Which one an eventual
    integrated model should use is an OPEN DECISION, documented in
    docs/DRIVETRAIN_PARAMETERS.md.

    49.4 and 58.4 ARE NOT RATIOS.  They are gear/gain constants that appear in the
    MyoAssist OpenSourceLeg_KA_L1 model, and they were never OSL V2 total reductions.
    The physical OSL V2 total ratio is 41.49 (paper) or 40.909 (our CAD).  Do not
    label either of them 49.4.

BACKLASH -- NOT MODELLED, BY THE PAPER'S OWN CHOICE
    Paper, Sec. III-A2, verbatim: "We further note that there appears to be a minor
    backlash behavior around zero deflection, but choose to neglect it for model
    simplicity."  So: backlash is OBSERVED in the hardware and DELIBERATELY EXCLUDED
    from the model.  No backlash model is invented here.  `rho` is therefore exactly
    continuous through theta_s = 0 with slope p1, and a deadband would be an
    unsupported addition, not a refinement.  See `BACKLASH_NOTE`.

SIGN FUNCTION AT ZERO
    The paper writes sgn(.) in (2) and (4).  At exactly zero, `sgn` here returns 0,
    which makes tau_f = 0 at theta_a_dot = 0 and tau_j = 0 at theta_s = 0 -- both
    continuous and both what the equations literally say.  Separately, the paper's own
    IMPLEMENTATION replaces sgn by a smooth sigma(x) = x/(|x| + alpha) with
    alpha = 5e-2 "to prevent limit cycles that could be caused by the discontinuous
    zero crossing in the friction model".  That smoothing is offered here as
    `smooth_sign` and as the optional `smooth_alpha` argument, and it is OFF by
    default.  Note honestly: the paper introduces sigma in its CONTROLLER
    implementation (Sec. III-B2 end), not as part of the plant model (1)-(2), so
    using it inside the plant is an extrapolation of the paper's device, flagged as
    such rather than presented as the paper's plant.

UNITS, EVERYWHERE, NO EXCEPTIONS
    angle rad, velocity rad/s, acceleration rad/s^2, torque N.m, current A,
    inertia kg.m^2, damping N.m.s/rad, stiffness N.m/rad, p2 N.m/rad^2.
    There are no degrees anywhere in this file.

DEPENDENCIES -- AND ONE HONEST CAVEAT ABOUT HOW YOU IMPORT IT
    This FILE imports only `math` and `dataclasses` from the standard library.  No
    MuJoCo, no numpy.  But `oslbench/__init__.py` eagerly imports the bench modules,
    which do pull in numpy, so

        from oslbench.control.drivetrain import PAPER          # drags in numpy via the package
        import oslbench.control.drivetrain                     # same

    To load it with genuinely nothing installed -- which is how the drivetrain tests
    run -- bypass the package:

        import importlib.util, sys
        spec = importlib.util.spec_from_file_location("drivetrain", ".../drivetrain.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules["drivetrain"] = mod        # dataclasses needs this BEFORE exec
        spec.loader.exec_module(mod)

    The `sys.modules` assignment is not optional: @dataclass looks the module up by
    name while the class body is being processed, and without it you get an
    AttributeError from inside dataclasses.py.  tests/test_drivetrain.py does exactly
    this, so the equations stay testable in a bare interpreter.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

__all__ = [
    # parameters and states
    "DrivetrainParameters", "PAPER", "CAD_BELT_RATIO", "PAPER_BELT_RATIO",
    "ActuatorState", "JointState", "DrivetrainState",
    # sign conventions
    "sign", "smooth_sign", "SMOOTH_ALPHA_PAPER", "BACKLASH_NOTE",
    # kinematics, paper (3) and (11)
    "joint_angle", "belt_deflection", "actuator_angle",
    # actuator, paper (1), (2), (6)
    "motor_torque", "friction_torque", "actuator_output_torque",
    "actuator_acceleration", "current_for_actuator_torque",
    # belt, paper (4), (5), (12)
    "rho", "belt_torque", "joint_torque_from_deflection", "belt_stiffness",
    "deflection_for_joint_torque", "belt_stiffness_at_torque",
    # the two sides of the spring, paper (7)
    "joint_torque_from_actuator", "actuator_load_torque_from_joint",
    "joint_torque_linearised",
    # rigid-limit reflections -- comparison only, see each docstring
    "reflect_actuator_inertia_to_joint", "reflect_actuator_damping_to_joint",
    "reflect_coulomb_friction_to_joint", "joint_friction_full",
]

# ----------------------------------------------------------------------------------
# The paper's own smoothing factor for the sign function (Sec. III-B2, last paragraph).
# Paper: sigma(x) = x/(abs(x) + alpha), alpha = 5e-2.
SMOOTH_ALPHA_PAPER = 5.0e-2

BACKLASH_NOTE = (
    "Best et al. 2025, Sec. III-A2: 'there appears to be a minor backlash behavior "
    "around zero deflection, but [we] choose to neglect it for model simplicity.' "
    "Backlash is therefore OBSERVED in hardware and DELIBERATELY EXCLUDED from this "
    "model.  rho() is continuous through theta_s = 0 with slope p1 = K_s(0).  No "
    "deadband is implemented, because inventing one would not be the paper's model."
)

# The two belt ratios, kept separate on purpose.  See the module docstring.
PAPER_BELT_RATIO = 4.61            # PAPER-DERIVED: Best et al. Sec. II, "4.61:1"
CAD_BELT_RATIO = 50.0 / 11.0       # CAD-DERIVED: our Onshape export's pulley teeth


def sign(x: float) -> float:
    """The paper's sgn(.), with sgn(0) = 0.

    Returning 0 at exactly zero is what (2) and (4) literally say, and it keeps both
    tau_f(0) and tau_j(0) equal to zero.  For the paper's own smooth replacement see
    `smooth_sign`.
    """
    if x > 0.0:
        return 1.0
    if x < 0.0:
        return -1.0
    return 0.0


def smooth_sign(x: float, alpha: float = SMOOTH_ALPHA_PAPER) -> float:
    """The paper's sigma(x) = x/(|x| + alpha), alpha = 5e-2 (Sec. III-B2).

    NOT a generic textbook tanh smoothing: this is the specific form the paper states
    it used.  The paper introduces it in the CONTROLLER implementation, to avoid limit
    cycles at the friction zero-crossing; applying it to the plant model is an
    extrapolation, so it is opt-in everywhere in this module.
    """
    if alpha <= 0.0:
        raise ValueError("alpha must be > 0; use sign() for the exact sgn")
    return x / (abs(x) + alpha)


# ==================================================================================
# PARAMETERS
# ==================================================================================
@dataclass(frozen=True)
class DrivetrainParameters:
    """Immutable OSL V2 drivetrain parameters.  Frozen so a caller cannot mutate the
    paper's values in place; use `dataclasses.replace` (or `.with_belt_ratio`) to make
    a clearly-labelled variant.

    Field                units          frame / side            source
    ------------------   ------------   ---------------------   -----------------------
    n_a                  -              rotor -> actuator out   PAPER (Sec. II, 9:1)
    n_t                  -              actuator out -> joint   PAPER (Sec. II, 4.61:1)
    k_t                  N.m/A          ROTOR                   PAPER (Sec. III-A1, fit)
    J_a                  kg.m^2         ACTUATOR OUTPUT         PAPER (Sec. III-A1, fit)
    B_a                  N.m.s/rad      ACTUATOR OUTPUT         PAPER (Sec. III-A1, fit)
    f_c                  N.m            ACTUATOR OUTPUT         PAPER (Sec. III-A1, fit)
    f_g                  N.m/A          ACTUATOR OUTPUT per     PAPER (Sec. III-A1, fit)
                                        ROTOR amp (mixed)
    p1                   N.m/rad        JOINT                   PAPER (Fig. 3 fit)
    p2                   N.m/rad^2      JOINT                   PAPER (Fig. 3 fit)

    Full provenance, including confidence and the open n_t decision, is in
    docs/DRIVETRAIN_PARAMETERS.md.
    """

    # --- gear ratios -------------------------------------------------------------
    n_a: float = 9.0               # [-]  planetary reduction INSIDE the actuator
    n_t: float = PAPER_BELT_RATIO  # [-]  single-stage belt reduction

    # --- actuator, all AT THE ACTUATOR OUTPUT SHAFT ------------------------------
    k_t: float = 110.8e-3          # [N.m/A]      rotor torque constant
    J_a: float = 9.83e-3           # [kg.m^2]     rotor+gearbox inertia AT THE OUTPUT
    B_a: float = 6.06e-2           # [N.m.s/rad]  viscous loss
    f_c: float = 17.1e-2           # [N.m]        coulomb friction
    f_g: float = 82.1e-3           # [N.m/A]      gear friction, per |I_q|

    # --- belt, AT THE JOINT ------------------------------------------------------
    p1: float = 876.0              # [N.m/rad]    linear term  == K_s(0)
    p2: float = 14913.0            # [N.m/rad^2]  quadratic (stiffening) term

    # ---------------------------------------------------------------- derived ----
    @property
    def total_ratio(self) -> float:
        """n_a * n_t -- the TOTAL NOMINAL MECHANICAL ratio, rotor to joint [-].

        4.61 belt -> 41.49.  This is emphatically not 49.4; see the module docstring.
        """
        return self.n_a * self.n_t

    @property
    def k_t_actuator(self) -> float:
        """Torque constant referred to the ACTUATOR OUTPUT: k_t * n_a [N.m/A]."""
        return self.k_t * self.n_a

    @property
    def k_t_joint(self) -> float:
        """Torque constant referred to the JOINT: k_t * n_a * n_t [N.m/A].

        RIGID, QUASI-STATIC ONLY.  This is the tau_j/I_q slope in the limit of no
        acceleration, no velocity and no compliance.  It is a convenient scale, not a
        model of the drivetrain.
        """
        return self.k_t * self.n_a * self.n_t

    @property
    def rotor_inertia_implied(self) -> float:
        """J_a / n_a^2 [kg.m^2] -- the rotor inertia IMPLIED by J_a.

        DERIVED, not measured: the paper reports only the lumped output-side J_a.
        Provided so that nobody is tempted to compute n_a^2 * J_a, which would
        double-count the planetary the paper already folded in.
        """
        return self.J_a / (self.n_a ** 2)

    def with_belt_ratio(self, n_t: float) -> "DrivetrainParameters":
        """A copy with a different belt ratio.  Explicit, so a swap is never silent."""
        return replace(self, n_t=float(n_t))

    # ------------------------------------------------------------ sanity check ----
    def validate(self) -> list[str]:
        """Return a list of complaints; empty list means every parameter is sane.

        Checks signs and physical plausibility only -- it cannot check that the values
        are the right ones, only that they are not nonsense.
        """
        bad: list[str] = []
        for name in ("n_a", "n_t", "k_t", "J_a", "B_a", "f_c", "f_g", "p1", "p2"):
            v = float(getattr(self, name))
            if not math.isfinite(v):
                bad.append(f"{name} is not finite ({v})")
            elif v <= 0.0:
                bad.append(f"{name} must be > 0 (got {v})")
        if not bad:
            # A stiffening spring needs p2 > 0; a softening one would be a sign error.
            if self.p2 <= 0.0:
                bad.append("p2 <= 0: the belt would soften with deflection, not stiffen")
            # The paper's friction compensation (6) requires k_t*n_a - f_g > 0, else the
            # denominator can vanish and the compensator is undefined.  Appendix A1:
            # "k_t n_a - sgn(I_q) sgn(theta_a_dot) f_g > 0 for all theta_a_dot, I_q".
            if self.k_t_actuator <= self.f_g:
                bad.append(f"k_t*n_a ({self.k_t_actuator:.4f}) <= f_g ({self.f_g:.4f}): "
                           "the paper's Appendix-A1 positivity condition fails")
        return bad


#: The paper's parameter set.  PAPER-DERIVED throughout -- not measured on our hardware.
PAPER = DrivetrainParameters()


# ==================================================================================
# STATE
# ==================================================================================
@dataclass(frozen=True)
class ActuatorState:
    """State of the ACTUATOR OUTPUT SHAFT (the belt's input pulley).

    theta_a       [rad]     angle of the actuator output shaft, post-planetary
    theta_a_dot   [rad/s]   its velocity

    Acceleration is NOT a state: it is an output of the dynamics, see
    `actuator_acceleration`.
    """

    theta_a: float = 0.0
    theta_a_dot: float = 0.0


@dataclass(frozen=True)
class JointState:
    """State of the PROSTHETIC JOINT (the belt's output pulley = knee/ankle axis).

    theta_j       [rad]     joint angle, as a joint encoder would read it
    theta_j_dot   [rad/s]   joint velocity
    """

    theta_j: float = 0.0
    theta_j_dot: float = 0.0


@dataclass(frozen=True)
class DrivetrainState:
    """The full mechanical state: actuator side AND joint side.

    WHY BOTH.  A RIGID drivetrain has ONE mechanical degree of freedom -- fix theta_j
    and theta_a = n_t * theta_j follows.  A COMPLIANT drivetrain has TWO, because the
    belt can stretch: theta_a and theta_j move independently and their mismatch is the
    deflection theta_s.  So the extra state the belt costs is real, and this class is
    where it lives.

    theta_s is NOT stored.  It is a DERIVED quantity, exactly as in the paper's (3),
    and storing it too would allow the three numbers to drift out of consistency.
    Read it from the `theta_s` property.
    """

    actuator: ActuatorState = ActuatorState()
    joint: JointState = JointState()

    # ------------------------------------------------------------------ kinematics
    def theta_s(self, p: DrivetrainParameters) -> float:
        """BELT DEFLECTION [rad], JOINT SIDE.  Paper (3) rearranged:

            theta_j = theta_a/n_t + theta_s   =>   theta_s = theta_j - theta_a/n_t
        """
        return self.joint.theta_j - self.actuator.theta_a / p.n_t

    def theta_s_dot(self, p: DrivetrainParameters) -> float:
        """Deflection RATE [rad/s], joint side -- the time derivative of (3)."""
        return self.joint.theta_j_dot - self.actuator.theta_a_dot / p.n_t


# ==================================================================================
# KINEMATICS  (paper eq. 3, and its inverse)
# ==================================================================================
def joint_angle(theta_a: float, theta_s: float, p: DrivetrainParameters) -> float:
    """Paper (3):  theta_j = theta_a/n_t + theta_s.

    theta_a [rad] actuator side; theta_s [rad] joint side; returns theta_j [rad].
    """
    return theta_a / p.n_t + theta_s


def belt_deflection(theta_j: float, theta_a: float, p: DrivetrainParameters) -> float:
    """Paper (3) solved for the deflection:  theta_s = theta_j - theta_a/n_t  [rad].

    Sign convention, stated plainly: theta_s > 0 means the JOINT leads the reduced
    actuator angle.  That is the case the paper's Fig. 3 characterises, produced in
    their experiment by holding theta_a = 0 and driving theta_j from 0.0 to 0.167 rad.
    """
    return theta_j - theta_a / p.n_t


def actuator_angle(theta_j: float, theta_s: float, p: DrivetrainParameters) -> float:
    """Paper (11)'s kinematic statement:  theta_a = n_t * (theta_j - theta_s)  [rad].

    Algebraically identical to (3); provided because the paper uses this arrangement
    when differentiating, and having both makes the round-trip testable.
    """
    return p.n_t * (theta_j - theta_s)


# ==================================================================================
# ACTUATOR  (paper eq. 1-2)
# ==================================================================================
def motor_torque(i_q: float, p: DrivetrainParameters) -> float:
    """Paper (1):  tau_m = I_q * k_t * n_a   [N.m at the ACTUATOR OUTPUT].

    i_q [A] is the q-axis current, a ROTOR-side quantity.  The n_a factor is what
    carries it across the planetary, so the result is an actuator-output torque.  The
    rotor's own torque would be i_q * k_t.
    """
    return i_q * p.k_t * p.n_a


def friction_torque(theta_a_dot: float, i_q: float, p: DrivetrainParameters,
                    smooth_alpha: float | None = None) -> float:
    """Paper (2):  tau_f = sgn(theta_a_dot) * (f_c + f_g*|I_q|)  [N.m at the ACTUATOR
    OUTPUT].

    Two components, and the paper names them separately:
        f_c          coulomb friction  [N.m]    -- constant magnitude
        f_g * |I_q|  gear friction     [N.m]    -- grows with current magnitude

    The magnitude depends on |I_q| but the DIRECTION is set by sgn(theta_a_dot) alone,
    so the result always OPPOSES motion (it enters (1) with a minus sign).  At
    theta_a_dot = 0 the exact sgn gives 0 -- the model has no static-friction breakaway
    term, and none is invented here.

    f_g is dimensionally mixed on purpose: it maps a ROTOR current in A to an
    ACTUATOR-OUTPUT torque in N.m.  Do not rescale it by n_a.

    smooth_alpha: if given, use the paper's sigma(x) = x/(|x|+alpha) instead of sgn.
    Off by default; see `smooth_sign` for why that is an extrapolation.
    """
    s = sign(theta_a_dot) if smooth_alpha is None else smooth_sign(theta_a_dot,
                                                                   smooth_alpha)
    return s * (p.f_c + p.f_g * abs(i_q))


def actuator_output_torque(i_q: float, theta_a_dot: float, theta_a_ddot: float,
                           p: DrivetrainParameters,
                           smooth_alpha: float | None = None) -> float:
    """Paper (1):  tau_a = tau_m - tau_f - B_a*theta_a_dot - J_a*theta_a_ddot
    [N.m at the ACTUATOR OUTPUT].

    This is the FORWARD reading of (1): given what the motor is doing and how the
    shaft is moving, how much torque actually leaves the actuator.  The three
    subtracted terms are the losses and the inertial cost -- they are why tau_a is not
    simply tau_m, and why n_t*tau_m is not the joint torque.
    """
    tau_m = motor_torque(i_q, p)
    tau_f = friction_torque(theta_a_dot, i_q, p, smooth_alpha)
    return tau_m - tau_f - p.B_a * theta_a_dot - p.J_a * theta_a_ddot


def actuator_acceleration(i_q: float, theta_a_dot: float, tau_a: float,
                          p: DrivetrainParameters,
                          smooth_alpha: float | None = None) -> float:
    """Paper (1) rearranged for SIMULATION:

        J_a*theta_a_ddot = tau_m - tau_f - B_a*theta_a_dot - tau_a
        =>  theta_a_ddot = (tau_m - tau_f - B_a*theta_a_dot - tau_a) / J_a

    [rad/s^2 at the ACTUATOR OUTPUT].

    tau_a here is the LOAD the belt imposes on the actuator, which in this drivetrain
    is not free: it is tau_j/n_t, fixed by the belt deflection (paper 4).  Use
    `actuator_load_torque_from_joint` to get it.  This function plus its joint-side
    counterpart are what an integrator would need -- but no integrator is provided
    here, because stepping is not this module's job.
    """
    tau_m = motor_torque(i_q, p)
    tau_f = friction_torque(theta_a_dot, i_q, p, smooth_alpha)
    return (tau_m - tau_f - p.B_a * theta_a_dot - tau_a) / p.J_a


def current_for_actuator_torque(tau_a_des: float, theta_a_dot: float,
                                p: DrivetrainParameters) -> float:
    """Paper (6), the friction-compensating current [A]:

        I_q = (tau_a_des + sgn(theta_a_dot)*f_c)
              / (k_t*n_a - sgn(tau_a_des*theta_a_dot + |theta_a_dot|*f_c) * f_g)

    Included because it is the exact inverse of (1)-(2) and therefore the sharpest
    available test of the friction model's algebra -- NOT because the paper's
    controller is being implemented here.  The rest of the compensator ((8), (16),
    (17), (20)) is deliberately absent.

    The denominator's positivity is guaranteed by the paper's Appendix-A1 condition
    k_t*n_a > f_g, which `DrivetrainParameters.validate` checks.
    """
    num = tau_a_des + sign(theta_a_dot) * p.f_c
    den = p.k_t_actuator - sign(tau_a_des * theta_a_dot
                                + abs(theta_a_dot) * p.f_c) * p.f_g
    return num / den


# ==================================================================================
# BELT / TRANSMISSION  (paper eq. 4-5, and 12)
# ==================================================================================
def rho(theta_s_abs: float, p: DrivetrainParameters) -> float:
    """Paper's fitted belt curve:  rho(theta_s) = p2*theta_s^2 + p1*theta_s  [N.m].

    DOMAIN.  The paper defines rho "for theta_s in R+", i.e. this function is the
    POSITIVE-deflection branch and its argument is a MAGNITUDE.  The extension to
    negative deflection is not done inside rho; it is done by (4), which wraps rho in
    sgn(.) and |.| -- see `belt_torque` / `joint_torque_from_deflection`.  Passing a
    negative value here is a caller error and raises.

    Fit quality, from the paper: R^2 = 0.997 over ten repeated trials.  Fig. 3 shows
    the fit exercised out to roughly 0.055 rad / ~93 N.m, so beyond that this is
    EXTRAPOLATION of their regression, not a region they measured.
    """
    if theta_s_abs < 0.0:
        raise ValueError(
            "rho() takes |theta_s|: the paper defines rho for theta_s in R+ and "
            "handles the sign in (4).  Use belt_torque() or "
            "joint_torque_from_deflection() for signed deflections."
        )
    return p.p2 * theta_s_abs ** 2 + p.p1 * theta_s_abs


def belt_torque(theta_s: float, p: DrivetrainParameters) -> float:
    """tau_s = sgn(theta_s) * rho(|theta_s|)  [N.m at the JOINT].

    "The torque in the transmission" (paper Sec. III-A2) -- the quantity their Fig. 3
    plots as "Belt Torque" against deflection.  Odd in theta_s, as a spring must be.
    The torque delivered to the JOINT is the NEGATIVE of this; see (4) and
    `joint_torque_from_deflection`.
    """
    return sign(theta_s) * rho(abs(theta_s), p)


def joint_torque_from_deflection(theta_s: float, p: DrivetrainParameters) -> float:
    """Paper (4):  tau_j = -tau_s = -sgn(theta_s) * rho(|theta_s|)  [N.m at the JOINT].

    Valid over the full domain theta_s in R.  The paper's justification for extending
    their positive-deflection fit to negative deflection is explicit: "Experiments
    with negative belt deflections yielded similar results, allowing us to write an
    expression for the torque applied at the ankle joint ... over a full domain."

    THE MINUS SIGN IS THE PHYSICS, NOT A TYPO.  theta_s > 0 means the joint has run
    ahead of the reduced actuator angle, stretching the belt; the belt then pulls the
    joint BACK, so tau_j < 0.  It is a restoring force.
    """
    return -sign(theta_s) * rho(abs(theta_s), p)


def belt_stiffness(theta_s: float, p: DrivetrainParameters) -> float:
    """Paper (5):  K_s(theta_s) = d rho / d theta_s = 2*p2*|theta_s| + p1
    [N.m/rad at the JOINT].

    The LOCAL stiffness: it rises linearly with |deflection|, which is what "the belt
    acts as a nonlinear spring, with stiffness increasing linearly with deflection"
    means.  At zero deflection it is p1 = 876 N.m/rad exactly, and it is even in
    theta_s (stretching either way stiffens the belt).
    """
    return 2.0 * p.p2 * abs(theta_s) + p.p1


def deflection_for_joint_torque(tau_j: float, p: DrivetrainParameters) -> float:
    """Paper (12), the inverse belt law:

        theta_s = rho^-1(tau_j) = -sgn(tau_j) * (-p1 + sqrt(p1^2 + 4*p2*|tau_j|)) / (2*p2)

    [rad at the JOINT].  The leading -sgn(tau_j) is required for consistency with (4):
    since tau_j = -sgn(theta_s)*rho(|theta_s|), a positive joint torque can only come
    from a NEGATIVE deflection.
    """
    mag = (-p.p1 + math.sqrt(p.p1 ** 2 + 4.0 * p.p2 * abs(tau_j))) / (2.0 * p.p2)
    return -sign(tau_j) * mag


def belt_stiffness_at_torque(tau_j: float, p: DrivetrainParameters) -> float:
    """K_s expressed directly in joint torque:  K_s = sqrt(p1^2 + 4*p2*|tau_j|)
    [N.m/rad].

    Not printed as a numbered equation in the paper, but it is (5) composed with (12)
    and the algebra is exact -- the 2*p2*|theta_s| and the -p1 inside (12) cancel:

        K_s = 2*p2*|theta_s| + p1
            = (-p1 + sqrt(p1^2 + 4*p2*|tau_j|)) + p1
            = sqrt(p1^2 + 4*p2*|tau_j|).

    The paper uses exactly this composition in practice -- Sec. III-B2: "We used the
    resulting tau_0 to calculate the expected deflection via rho^-1 and the
    corresponding local transmission stiffness K_s using (5)."  This function is a
    shortcut for that two-step, and `check_drivetrain_model.py` verifies the two
    routes agree.
    """
    return math.sqrt(p.p1 ** 2 + 4.0 * p.p2 * abs(tau_j))


# ==================================================================================
# THE TWO SIDES OF THE SPRING  (paper eq. 7 and its surrounding text)
# ==================================================================================
def joint_torque_from_actuator(tau_a: float, p: DrivetrainParameters) -> float:
    """tau_j = n_t * tau_a  [N.m at the JOINT].

    The paper's statement, Sec. III-B1: "The actuator torque and the joint torque are
    related by tau_j = n_t tau_a."  It holds because the belt is modelled as MASSLESS,
    so torque in equals torque out up to the ratio.

    n_a does NOT appear.  It is already inside tau_a, having been applied in (1) via
    tau_m = I_q*k_t*n_a.  Multiplying by n_a here would double-count it.
    """
    return p.n_t * tau_a


def actuator_load_torque_from_joint(tau_j: float, p: DrivetrainParameters) -> float:
    """tau_a = tau_j / n_t  [N.m at the ACTUATOR OUTPUT] -- the previous function
    inverted.

    This is how the belt's spring law becomes a LOAD on the actuator: compute tau_j
    from the deflection with (4), divide by n_t, and feed it to
    `actuator_acceleration` as tau_a.
    """
    return tau_j / p.n_t


def joint_torque_linearised(tau_a_des: float, theta_a_dot: float, theta_a_ddot: float,
                            p: DrivetrainParameters) -> float:
    """Paper (7):  tau_j = n_t * (tau_a_des - B_a*theta_a_dot - J_a*theta_a_ddot)
    [N.m at the JOINT].

    The chain AFTER perfect friction compensation, i.e. (1) with tau_f cancelled by
    (6).  Note what survives the cancellation: the actuator's viscous and inertial
    terms, multiplied by n_t.  Those two terms are exactly what our MuJoCo bench's
    single joint-side `armature`/`damping` pair is trying to stand in for, and this
    line is where you can see the n_t (not n_a*n_t, and not 49.4) that belongs in
    front of them.
    """
    return p.n_t * (tau_a_des - p.B_a * theta_a_dot - p.J_a * theta_a_ddot)


# ==================================================================================
# JOINT-SIDE REFLECTIONS -- USEFUL, AND EASY TO MISUSE
# ==================================================================================
def reflect_actuator_inertia_to_joint(p: DrivetrainParameters) -> float:
    """n_t^2 * J_a  [kg.m^2 at the JOINT].

    WHAT THIS IS.  The joint-side equivalent of the actuator's inertia in the RIGID
    limit -- the number you would put in a MuJoCo `armature` if you decided to
    approximate the whole compliant drivetrain by one rigid joint-side inertia.

    WHAT THIS IS NOT.  It is NOT a consequence of the paper's model; the paper never
    forms this product.  It is the rigid-drivetrain approximation OF the paper's
    model, and it discards the belt entirely.  Three specific things it throws away:
    the compliance (theta_a and theta_j become one DOF again), the current-dependent
    part of the friction, and the distinction between actuator-side and joint-side
    velocity.  Use it for comparison, label it "rigid equivalent", and do not call it
    the paper's actuator model.

    And do NOT reach for n_a^2 * n_t^2 * J_a: the n_a^2 is already inside J_a.
    """
    return (p.n_t ** 2) * p.J_a


def reflect_actuator_damping_to_joint(p: DrivetrainParameters) -> float:
    """n_t^2 * B_a  [N.m.s/rad at the JOINT].  Rigid-limit equivalent; same caveats as
    `reflect_actuator_inertia_to_joint`.  The n_t^2 (not n_t) is because damping maps
    like an impedance: torque scales by n_t and the velocity it multiplies scales by
    1/n_t."""
    return (p.n_t ** 2) * p.B_a


def reflect_coulomb_friction_to_joint(p: DrivetrainParameters) -> float:
    """n_t * f_c  [N.m at the JOINT].  Rigid-limit equivalent of the CONSTANT part of
    the friction ONLY.

    n_t, not n_t^2: f_c is a torque, so it scales once.  This is a strictly incomplete
    picture of the friction, because it leaves out f_g*|I_q| -- which, at the torques
    our bench actually reaches, is the LARGER of the two terms.  See
    `joint_friction_full` and docs/DRIVETRAIN_PARAMETERS.md.
    """
    return p.n_t * p.f_c


def joint_friction_full(theta_a_dot: float, i_q: float,
                        p: DrivetrainParameters) -> float:
    """n_t * sgn(theta_a_dot) * (f_c + f_g*|I_q|)  [N.m at the JOINT] -- the WHOLE
    friction, reflected.

    Provided next to `reflect_coulomb_friction_to_joint` so the gap between them is
    visible: this one moves with the operating point, the other cannot.  A single
    MuJoCo `frictionloss` scalar can represent the second but not the first.
    """
    return p.n_t * friction_torque(theta_a_dot, i_q, p)
