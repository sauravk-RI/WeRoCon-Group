"""LegSimulation -- the coupled knee + ankle MuJoCo plant used for every gait test.

WHAT THIS IS
    A re-implementation of the project's validated ``AssembledTracking`` MuJoCo plant
    with its FINAL PAPER configuration (``replay_paper.py`` in that research codebase)
    built in, instead of being patched on afterwards:

      * model            models/osl_v2_bench_aluminium_2810.xml (5.069 kg fixed-base leg)
      * physics          0.5 ms MuJoCo step, implicitfast, gravity, no ground / no GRF
      * actuators        both authored MuJoCo position servos DISABLED; each joint is
                         driven only by its own compliant drivetrain torque written into
                         qfrc_applied (Best et al. actuator + nonlinear belt law)
      * drivetrain       belt ratio 4.61, output KT 0.7044 N.m/A, rotor KT 0.095 N.m/A,
                         current-dependent gear friction 0.0821 N.m per rotor-A
      * controller       two independent current-domain PIDs (deg in, A out) at 200 Hz,
                         +/-60 A clip, ideal current loop, integral limit 50 deg.s,
                         "Configuration C" derivative (target minus gearbox-output angle
                         referred to the joint) -- the final paper setting
      * gains            knee 3.0 / 0 / 0.10, ankle 2.0 / 0 / 0.075 (Kp, Ki, Kd) unless the
                         hardware reports different ones in sim-to-real mode
      * ankle range      runtime range widened to +/-31 deg (the research code's
                         ``--ankle-range hardware``); the XML on disk is unchanged

    The stepping loop (``advance``) and the replay pairing (``accept``) are line-for-line
    the same algorithm as the reference implementation. ``validate_engine.py`` replays
    six recorded gait trials through THIS class and checks the angle RMSE against the
    published results/gait_summary.csv.

ANGLE FRAME ("sim frame")
    All angles in and out of this class are degrees in the MuJoCo joint coordinates:
        knee   positive = flexion        (hardware encoder is flexion-NEGATIVE -> x -1)
        ankle  positive = plantarflexion (same sign as the hardware ankle encoder)
    This is exactly the frame of the hardware scripts' CSV output (they flip the knee
    "cosmetically" when writing), and of the reference telemetry's angle multipliers.
"""
from __future__ import annotations

import hashlib
import math
from collections import deque
from dataclasses import replace

import numpy as np
import mujoco

from .paths import MODEL_PATH
from .reference_control.current import (CurrentLimits, PIDGains, ConversionAudit,
                                        CAD_BELT_RATIO, ANKLE_PAPER_CAD_BELT)
from .reference_control.gait import JointCurrentConfig, make_gait_source
from .reference_control.compliance import DrivetrainLayer

from .frames import JOINTS, SIM_FRAME_SIGN          # noqa: F401  (re-exported)

# ---- the research codebase's final paper settings (replay_paper.py) -------------------
BELT_RATIO = 4.61
KT_OUTPUT_NM_PER_A = 0.7044
ROTOR_KT_NM_PER_A = 0.095
PAPER_FG = 0.0821
CURRENT_CLIP_A = 60.0
INTEGRAL_LIMIT_DEG_S = 50.0
FEEDBACK = "actuator-difference"          # Configuration C
CONTROL_HZ = 200.0
PHYSICS_DT = 0.0005
ANKLE_RUNTIME_RANGE_DEG = (-31.0, 31.0)
BELT_FIT_EDGE_RAD = 0.055
BELT_DIVERGENCE_RAD = 0.15
DEFAULT_GAINS = {"knee": (3.0, 0.0, 0.10), "ankle": (2.0, 0.0, 0.075)}


def apply_final_paper_drivetrain(source, layer):
    """The reference implementation's ``configure()`` from replay_paper.py, verbatim in effect."""
    old = source.audit
    assert old.params.f_g == PAPER_FG
    params = replace(old.params, k_t=ROTOR_KT_NM_PER_A, n_t=BELT_RATIO)
    source.audit = replace(old, k_t_output_Nm_per_A=KT_OUTPUT_NM_PER_A, belt_ratio=BELT_RATIO,
                           params=params, belt_ratio_source="Final paper setting",
                           k_t_output_provenance="CubeMars V3 driver manual")
    layer.p = params
    assert source.loop.ideal


class SimulationDiverged(RuntimeError):
    pass


class LegSimulation:
    """Two current PIDs + two compliant drivetrains acting on ONE coupled MuJoCo leg.

    initial_deg      {'knee': deg, 'ankle': deg}  starting joint angles (sim frame)
    initial_ref_deg  {'knee': deg, 'ankle': deg}  first reference (sim frame)
    gains            {'knee': (kp, ki, kd), 'ankle': (...)}  A/deg, A/(deg.s), A/(deg/s)
    on_step          callback(row) for every physics step and joint (full-rate log)
    origin           time of the first physics step (s)
    """

    def __init__(self, initial_deg, initial_ref_deg, gains=None, on_step=None,
                 origin=0.0, physics_dt=PHYSICS_DT, model_path=MODEL_PATH,
                 ankle_range_deg=ANKLE_RUNTIME_RANGE_DEG, history_len=200):
        gains = dict(DEFAULT_GAINS if gains is None else gains)
        if set(gains) != set(JOINTS) or any(
                len(g) != 3 or not all(math.isfinite(v) and v >= 0 for v in g)
                for g in gains.values()):
            raise ValueError("Provide three finite, nonnegative PID gains for each joint")
        self.gains = {j: tuple(float(v) for v in gains[j]) for j in JOINTS}
        self.model_path = str(model_path)
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        if float(self.model.opt.timestep) != PHYSICS_DT or self.model.nq != 2:
            raise ValueError("Unexpected full-leg model (need nq=2, 0.5 ms timestep)")
        steps = (1.0 / CONTROL_HZ) / physics_dt
        if physics_dt <= 0 or abs(steps - round(steps)) > 1e-8:
            raise ValueError("Physics timestep must divide the 5 ms control period")
        self.dt = float(physics_dt)
        self.model.opt.timestep = self.dt
        self.steps_per_update = int(round(steps))
        self.qpos = {j: int(self.model.joint(j).qposadr[0]) for j in JOINTS}
        self.dof = {j: int(self.model.joint(j).dofadr[0]) for j in JOINTS}
        self.original_ankle_range_rad = self.model.joint("ankle").range.copy().tolist()
        self.model.joint("ankle").range[:] = np.radians(ankle_range_deg)
        # Disable BOTH authored position servos. Only explicit belt torque drives the leg.
        self.model.actuator_gainprm[:] = 0.0
        self.model.actuator_biasprm[:] = 0.0

        self.origin = float(origin)
        self.k = 0
        self.on_step = on_step or (lambda row: None)
        self.fit_exceeded = 0
        self.sources, self.layers = {}, {}
        for j in JOINTS:
            self.data.qpos[self.qpos[j]] = math.radians(float(initial_deg[j]))
            limits = CurrentLimits(sim_clip_A=CURRENT_CLIP_A)
            audit = ConversionAudit(.5701, "script", "Provisional actuator-output Nm/A",
                                    CAD_BELT_RATIO, "CAD50/11", ANKLE_PAPER_CAD_BELT, limits)
            cfg = JointCurrentConfig(
                j, PIDGains(*self.gains[j], encoder_to_motor_sign=1), limits, 1,
                "positive CAD coordinate; sim-frame angle mapped directly",
                "Hardware gains", "positive simulated torque; physical mapping provisional",
                integral_limit_deg_s=INTEGRAL_LIMIT_DEG_S)
            if not cfg.negative_feedback:
                raise ValueError("Positive feedback is forbidden in the simulation adapter")
            layer = DrivetrainLayer(audit.params, enabled=True, substeps=1)
            source = make_gait_source(cfg, audit,
                                      np.array([math.radians(float(initial_ref_deg[j]))]),
                                      self.steps_per_update, layer, FEEDBACK)
            source.dt = self.dt
            apply_final_paper_drivetrain(source, layer)
            layer.seed_rigid(float(self.data.qpos[self.qpos[j]]), 0.0)
            self.sources[j], self.layers[j] = source, layer
        mujoco.mj_forward(self.model, self.data)
        self.history = deque([(self.origin, self.angles())], maxlen=history_len)
        self.last = {}

    # ------------------------------------------------------------------ state
    def angles(self):
        return {j: math.degrees(float(self.data.qpos[self.qpos[j]])) for j in JOINTS}

    @property
    def time(self):
        return self.origin + self.k * self.dt

    def joint_range_deg(self, j):
        lo, hi = np.degrees(self.model.joint(j).range)
        return float(lo), float(hi)

    def set_reference(self, j, ref_deg):
        lo, hi = self.joint_range_deg(j)
        if not lo <= ref_deg <= hi:
            raise ValueError(f"{j} target {ref_deg:.3f} deg outside runtime range "
                             f"[{lo:.1f}, {hi:.1f}]; no target clipping")
        self.sources[j].ref[0] = math.radians(ref_deg)

    # ------------------------------------------------------------------ stepping
    def advance(self, t):
        """Step physics until the simulation clock reaches t (reference implementation's algorithm)."""
        while self.origin + self.k * self.dt < t - 1e-10:
            rows = []
            for j in JOINTS:
                q = float(self.data.qpos[self.qpos[j]])
                qd = float(self.data.qvel[self.dof[j]])
                src, layer = self.sources[j], self.layers[j]
                current = src(self.k, q, qd)
                rec = layer.advance(q, current, self.dt)
                self.data.qfrc_applied[self.dof[j]] = rec.tau_j_mean
                fit = abs(rec.theta_s) > BELT_FIT_EDGE_RAD
                self.fit_exceeded += int(fit)
                if abs(rec.theta_s) > BELT_DIVERGENCE_RAD:
                    raise SimulationDiverged("Belt deflection >0.15 rad: divergent/out-of-fit "
                                             "simulation stopped")
                s = src.last
                rows.append(dict(
                    joint=j, source_time_s=self.origin + (self.k + 1) * self.dt,
                    target_deg=math.degrees(src.ref[0]),
                    current_requested_A=s["current_request_A"],
                    current_applied_A=s["current_applied_A"],
                    current_clipped=s["saturated"],
                    pid_p_A=s["p_A"], pid_i_A=s["i_A"], pid_d_A=s["d_A"],
                    joint_drive_torque_Nm=rec.tau_j, belt_actuator_torque_Nm=rec.tau_a,
                    actuator_torque_Nm=rec.tau_m, friction_torque_Nm=rec.tau_f,
                    belt_deflection_rad=rec.theta_s, belt_energy_J=rec.U,
                    belt_stiffness_Nm_per_rad=rec.K_s, actuator_velocity_rad_s=rec.theta_a_dot,
                    joint_velocity_rad_s=qd, belt_fit_exceeded=int(fit)))
            mujoco.mj_step(self.model, self.data)
            self.k += 1
            if not np.isfinite(self.data.qpos).all():
                raise SimulationDiverged("Nonfinite MuJoCo state")
            angles = self.angles()
            self.history.append((self.origin + self.k * self.dt, angles))
            for row in rows:
                row["sim_angle_deg"] = angles[row["joint"]]
                self.last[row["joint"]] = row
                self.on_step(row)

    def control_tick(self, i, ref_deg):
        """Live/standalone mode: one 5 ms controller period starting at origin + i*5 ms.

        The reference for each joint is set first, then physics runs to the end of the
        period; the PID updates on the first physics step of the period (k % 10 == 0),
        exactly as in the reference implementation's tick-driven replay.
        """
        for j in JOINTS:
            self.set_reference(j, ref_deg[j])
        self.advance(self.origin + (i + 1) * self.steps_per_update * self.dt)

    # ------------------------------------------------------------------ replay
    def accept(self, tick):
        """Replay one recorded hardware tick (reference implementation's algorithm, used for validation)."""
        self.advance(tick["reference_time_s"])
        for j in JOINTS:
            self.set_reference(j, tick["joints"][j]["reference_deg"])
        self.advance(max(v["time_s"] for v in tick["joints"].values()))
        h = list(self.history)
        result = []
        for j in JOINTS:
            r = tick["joints"][j]
            t = r["time_s"]
            angle = float(np.interp(t, [x[0] for x in h], [x[1][j] for x in h]))
            hw = r["actual_deg"] if r.get("encoder_valid", True) else None
            result.append(dict(
                joint=j, source_time_s=t, reference_time_s=tick["reference_time_s"],
                target_deg=r["reference_deg"], hardware_angle_deg=hw, sim_angle_deg=angle,
                difference_deg=angle - hw if hw is not None else None,
                hardware_tracking_error_deg=r["reference_deg"] - hw if hw is not None else None,
                sim_tracking_error_deg=r["reference_deg"] - angle))
        return result

    # ------------------------------------------------------------------ metadata
    def metadata(self):
        return dict(
            engine="osl_sim.engine.LegSimulation (re-implementation of the reference "
                   "AssembledTracking + replay_paper final configuration)",
            model=self.model_path,
            model_sha256=hashlib.sha256(open(self.model_path, "rb").read()).hexdigest(),
            mujoco_version=mujoco.__version__,
            physics_dt_s=self.dt, control_hz=CONTROL_HZ,
            gains_Kp_Ki_Kd=self.gains, feedback=FEEDBACK + " (Configuration C)",
            hardware_derivative_note="hardware PID differentiates the joint-angle error; "
                                     "the simulation uses Configuration C (the reference paper "
                                     "setting) because the joint-error D diverges in the model",
            belt_ratio=BELT_RATIO, kt_output_Nm_per_A=KT_OUTPUT_NM_PER_A,
            rotor_kt_Nm_per_A=ROTOR_KT_NM_PER_A, gear_friction_fg=PAPER_FG,
            current_clip_A=CURRENT_CLIP_A, integral_limit_deg_s=INTEGRAL_LIMIT_DEG_S,
            current_loop="ideal",
            original_ankle_range_rad=self.original_ankle_range_rad,
            runtime_ankle_range_deg=list(self.joint_range_deg("ankle")),
            knee_range_deg=list(self.joint_range_deg("knee")),
            angle_frame="sim frame: knee +flexion (hardware encoder x -1), "
                        "ankle +plantarflexion (hardware encoder x +1)",
            assumptions="Root knee_prox fixed to world; no ground contact / body weight; "
                        "physical encoder zero/direction and mount orientation unverified; "
                        "drivetrain parameters from Best et al. / CubeMars, not fitted",
            belt_fit_exceeded_steps=self.fit_exceeded)
