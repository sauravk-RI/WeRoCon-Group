"""Engine-independent compliant drivetrain state and current sources."""
from __future__ import annotations
import math
from . import drivetrain as D
from .drivetrain import PAPER, DrivetrainParameters


def belt_potential_energy(theta_s: float, p: DrivetrainParameters = PAPER) -> float:
    """U(theta_s) = p2*|theta_s|^3/3 + p1*theta_s^2/2   [J], the belt's stored energy.

    This is the potential whose gradient is the paper's own belt law, and it is the
    only quantity in the model that makes an energy-conservation test meaningful.
    The derivation is one line and worth keeping here because a sign error in it would
    silently invalidate the drift test:

        dU/dtheta_s = p2*theta_s*|theta_s| + p1*theta_s
                    = sgn(theta_s) * (p2*theta_s^2 + p1*|theta_s|)
                    = sgn(theta_s) * rho(|theta_s|)
                    = -tau_j            by paper (4).

    So tau_j = -dU/dtheta_s, and because theta_s = theta_j - theta_a/n_t,

        Q_j = -dU/dtheta_j = tau_j          (the joint feels +tau_j)
        Q_a = -dU/dtheta_a = -tau_j/n_t     (the actuator feels -tau_a)

    which is exactly the pair of couplings implemented below.  U >= 0 always, U(0) = 0,
    and U is even in theta_s.
    """
    a = abs(theta_s)
    return p.p2 * a ** 3 / 3.0 + p.p1 * theta_s ** 2 / 2.0


def frictionless_parameters(p: DrivetrainParameters = PAPER) -> DrivetrainParameters:
    """A copy of `p` with B_a = f_c = f_g = 0 -- the energy-drift test's parameter set.

    Every dissipative term in the paper's actuator model is removed and NOTHING else
    changes, so the remaining system is conservative by construction and any measured
    energy change is numerical rather than physical.  This is a TEST FIXTURE.  It is
    not a claim that the OSL V2 drivetrain is frictionless.
    """
    return DrivetrainParameters(
        n_a=p.n_a, n_t=p.n_t, k_t=p.k_t, J_a=p.J_a,
        B_a=0.0, f_c=0.0, f_g=0.0,
        p1=p.p1, p2=p.p2,
    )


# ----------------------------------------------------------------- small records
class BeltState:
    """The belt evaluated at one configuration.  Pure function of (theta_j, theta_a)."""

    __slots__ = ("theta_s", "tau_j", "tau_a", "K_s", "U")

    def __init__(self, theta_s: float, tau_j: float, tau_a: float,
                 K_s: float, U: float):
        self.theta_s, self.tau_j, self.tau_a = theta_s, tau_j, tau_a
        self.K_s, self.U = K_s, U

    def __repr__(self) -> str:                                  # pragma: no cover
        return (f"BeltState(theta_s={self.theta_s:+.9e} rad, tau_j={self.tau_j:+.9f} "
                f"N.m, tau_a={self.tau_a:+.9f} N.m, K_s={self.K_s:.6f} N.m/rad, "
                f"U={self.U:.9e} J)")


class LayerStep:
    """What one `DrivetrainLayer.advance` call did.  Diagnostics, not physics.

    TIME STAMPS MATTER HERE, so they are recorded explicitly rather than implied.
    One semi-implicit sub-step reads some quantities BEFORE the update and some AFTER,
    and mixing them silently would make the consistency residuals in
    `experiments/layer_dynamic_test.py` look like physics errors when they are only
    bookkeeping errors:

        theta_j         the joint angle the belt was evaluated at (pre-mj_step)
        theta_a_pre     actuator angle BEFORE the sub-step  -> pairs with theta_s
        theta_a         actuator angle AFTER  the sub-step
        theta_a_dot_pre velocity BEFORE  -> the argument tau_f was evaluated at
        theta_a_dot     velocity AFTER   -> the velocity the implicit B_a term used
        theta_a_ddot    (theta_a_dot - theta_a_dot_pre)/dt, the realised acceleration
        theta_s, tau_j, tau_a, K_s, U      all at (theta_j, theta_a_pre)
        tau_m           at i_q;  tau_f at (theta_a_dot_pre, i_q)

    With substeps > 1 the `*_pre` fields belong to the LAST sub-step, which is the only
    one the returned belt state describes either.  substeps == 1 is the verified
    configuration (see docs/DRIVETRAIN_INTEGRATION_OPTIONS.md), where it is exact.
    """

    __slots__ = ("tau_j", "tau_j_mean", "tau_a", "theta_s", "theta_a", "theta_a_dot",
                 "tau_m", "tau_f", "i_q", "K_s", "U",
                 "theta_j", "theta_a_pre", "theta_a_dot_pre", "theta_a_ddot")

    def __init__(self, **kw):
        for key, val in kw.items():
            setattr(self, key, val)


# --------------------------------------------------------------------- THE LAYER
class DrivetrainLayer:
    """The actuator shaft, its integrator, and the belt coupling.  NO MuJoCo.

        p           the frozen paper parameters (or a variant, e.g. frictionless)
        enabled     False -> the layer is inert; nothing may be written anywhere
        substeps    actuator sub-steps per outer step (see the module docstring)

    State
        theta_a      rad     actuator OUTPUT shaft angle (post-planetary, pre-belt)
        theta_a_dot  rad/s   its velocity

    The class is deliberately dumb about time: `advance` is told dt rather than owning
    a clock, so the same object can be driven by MuJoCo, by a bare loop, or by a test.
    """

    def __init__(self, p: DrivetrainParameters = PAPER, enabled: bool = False,
                 substeps: int = 1):
        if substeps < 1:
            raise ValueError("substeps must be >= 1")
        self.p = p
        self.enabled = bool(enabled)
        self.substeps = int(substeps)
        self.theta_a = 0.0
        self.theta_a_dot = 0.0
        self.steps = 0

    # ------------------------------------------------------------------ seeding
    def seed_rigid(self, theta_j: float, theta_j_dot: float = 0.0) -> None:
        """Place the actuator where a RIGID belt would put it: theta_s = 0 exactly.

        theta_a = n_t*theta_j is the zero-deflection, zero-stored-energy state.  It is
        the right initial condition for any run that should not begin with the belt
        already wound up.
        """
        self.theta_a = self.p.n_t * float(theta_j)
        self.theta_a_dot = self.p.n_t * float(theta_j_dot)

    def seed_from_joint_torque(self, theta_j: float, tau_j: float,
                               theta_j_dot: float = 0.0) -> float:
        """Wind the belt up to a chosen joint torque and return the deflection used.

        Inverts paper (12) to get theta_s, then (3) to get the theta_a that produces
        it.  This is how the drift test "initialises a belt deflection" in units that
        mean something physically (a torque) rather than an arbitrary angle.
        """
        theta_s = D.deflection_for_joint_torque(float(tau_j), self.p)
        self.theta_a = self.p.n_t * (float(theta_j) - theta_s)
        self.theta_a_dot = self.p.n_t * float(theta_j_dot)
        return theta_s

    def seed_deflection(self, theta_j: float, theta_s: float,
                        theta_j_dot: float = 0.0) -> None:
        """Set an explicit belt deflection theta_s at joint angle theta_j."""
        self.theta_a = self.p.n_t * (float(theta_j) - float(theta_s))
        self.theta_a_dot = self.p.n_t * float(theta_j_dot)

    # ------------------------------------------------------------------- reading
    def belt(self, theta_j: float) -> BeltState:
        """Evaluate the belt at the CURRENT (theta_j, theta_a).  No state changes.

        Every number here comes from `oslbench/control/drivetrain.py`; nothing is re-derived.
        """
        p = self.p
        theta_s = D.belt_deflection(float(theta_j), self.theta_a, p)   # paper (3)
        tau_j = D.joint_torque_from_deflection(theta_s, p)             # paper (4)
        tau_a = D.actuator_load_torque_from_joint(tau_j, p)            # paper (7)
        return BeltState(theta_s, tau_j, tau_a,
                         D.belt_stiffness(theta_s, p),                 # paper (5)
                         belt_potential_energy(theta_s, p))

    def kinetic_energy(self) -> float:
        """0.5*J_a*theta_a_dot^2 [J] -- the ACTUATOR's kinetic energy only.

        The joint side's kinetic energy belongs to whoever owns the joint (MuJoCo, or
        the caller in a MuJoCo-free test), so it is deliberately not included here.
        """
        return 0.5 * self.p.J_a * self.theta_a_dot ** 2

    # -------------------------------------------------------------- THE INTEGRATOR
    def _advance_once(self, theta_j: float, i_q: float, dt: float):
        """One semi-implicit (symplectic) Euler sub-step of the ACTUATOR shaft.

            J_a*theta_a_ddot = tau_m - tau_f - B_a*theta_a_dot - tau_j/n_t

        Damping is taken IMPLICITLY because B_a/J_a = 6.166 1/s makes the explicit
        damping step dt*B_a/J_a = 3.08e-3 at h = 0.5 ms -- small, but treating it
        implicitly is unconditionally stable and costs one division:

            v_new = (v + dt*(tau_m - tau_f - tau_a)/J_a) / (1 + dt*B_a/J_a)
            x_new = x + dt*v_new

        The spring term tau_a is EXPLICIT, at the old configuration, on purpose: that
        is what makes this scheme match what MuJoCo does with `qfrc_applied`, and it is
        what makes the pair of them symplectic together.  See the module docstring.
        """
        p = self.p
        st = self.belt(theta_j)
        tau_m = D.motor_torque(i_q, p)                                 # paper (1)
        tau_f = D.friction_torque(self.theta_a_dot, i_q, p)            # paper (2)
        x_pre, v_pre = self.theta_a, self.theta_a_dot
        acc_num = tau_m - tau_f - st.tau_a
        self.theta_a_dot = ((self.theta_a_dot + dt * acc_num / p.J_a)
                            / (1.0 + dt * p.B_a / p.J_a))
        self.theta_a += dt * self.theta_a_dot
        return st, tau_m, tau_f, x_pre, v_pre

    def advance(self, theta_j: float, i_q: float, dt: float) -> LayerStep:
        """Advance the actuator by dt and return the joint torque MuJoCo should apply.

        The returned `tau_j_mean` is the number to write into `qfrc_applied`: with
        substeps == 1 it is exactly the tau_j used internally, and with substeps > 1 it
        is the time-average over the sub-steps so that the IMPULSE delivered to the
        joint over the outer step is right.

        theta_j is held FIXED across sub-steps because MuJoCo has not moved the joint
        yet -- it moves when mj_step runs, after this returns.
        """
        n = self.substeps
        h = dt / n
        acc = 0.0
        st = tau_m = tau_f = None
        x_pre = v_pre = 0.0
        for _ in range(n):
            st, tau_m, tau_f, x_pre, v_pre = self._advance_once(theta_j, i_q, h)
            acc += st.tau_j
        self.steps += 1
        return LayerStep(tau_j=st.tau_j, tau_j_mean=acc / n, tau_a=st.tau_a,
                         theta_s=st.theta_s, theta_a=self.theta_a,
                         theta_a_dot=self.theta_a_dot, tau_m=tau_m, tau_f=tau_f,
                         i_q=float(i_q), K_s=st.K_s, U=st.U,
                         theta_j=float(theta_j), theta_a_pre=x_pre,
                         theta_a_dot_pre=v_pre,
                         theta_a_ddot=(self.theta_a_dot - v_pre) / h)


# ------------------------------------------------------------------ CURRENT SOURCES
# A current source is any callable (k, theta_j, theta_j_dot) -> I_q amps.  These three
# cover every input this phase needs.  None of them is a tuned controller; the first two
# are open-loop waveforms for the plant test, and the third is the EXISTING PD law
# re-pointed at the current input instead of at the joint.

class ConstantCurrent:
    """I_q = const, for every step.  The simplest possible plant excitation."""

    def __init__(self, amps: float):
        self.amps = float(amps)

    def __call__(self, k: int, theta_j: float, theta_j_dot: float) -> float:
        return self.amps

    def __repr__(self) -> str:                                  # pragma: no cover
        return f"ConstantCurrent({self.amps:+.4f} A)"


class StepCurrent:
    """I_q = `first` until t >= `t_switch`, then `second`.  Switching is on STEP INDEX,

    computed as round(t_switch/dt), so the transition lands on an exact step boundary
    and the trace is reproducible independently of floating-point time accumulation.
    """

    def __init__(self, first: float, second: float, t_switch: float, dt: float):
        self.first, self.second = float(first), float(second)
        self.t_switch, self.dt = float(t_switch), float(dt)
        self.k_switch = int(round(float(t_switch) / float(dt)))

    def __call__(self, k: int, theta_j: float, theta_j_dot: float) -> float:
        return self.first if k < self.k_switch else self.second

    def __repr__(self) -> str:                                  # pragma: no cover
        return (f"StepCurrent({self.first:+.4f} -> {self.second:+.4f} A at "
                f"t = {self.t_switch:.4f} s = step {self.k_switch})")
