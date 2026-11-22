"""Run ONE gait test: standalone simulation (SIM_2_REAL = 0) or sim-to-real (= 1).

Called by the five sim_knee_ankle_trajectory_<gait>.py scripts with their USER SETTINGS.

SIM_2_REAL = 0   build the reference from the local dataset copy, simulate it in real time
                 (with the MuJoCo viewer if SHOW_VIEWER), log, plot. No network.

SIM_2_REAL = 1   1. connect to the leg Pi (it must be running the SAME gait script with
                    its own SIM_2_REAL = 1; order of starting does not matter)
                 2. receive the leg's resolved reference/gains/limits and compare them
                    with the local settings (the leg's are used -- it is the real system)
                 3. [Enter] = ARM  -> the leg powers its motors, homes and settles
                 4. leg READY      -> the simulated leg is placed at the leg's measured angles
                 5. [Enter] = START -> clocks synchronised, common start instant scheduled
                    GO_LEAD_S ahead; simulation and leg begin together
                 6. the leg streams its data; both are logged into the same run folder
                 Ctrl+C at any time sends STOP; a lost link also stops the leg.
"""
from __future__ import annotations

import math
import platform
import sys
import time
import traceback

import numpy as np

from . import __version__
from .engine import (CONTROL_HZ, JOINTS, SIM_FRAME_SIGN, LegSimulation, SimulationDiverged)
from .reference import GaitReference, build_reference
from .results import (HARDWARE_HEADER, hardware_rows, make_run_dir, pair_with_simulation,
                      plot_comparison, summarize, write_csv, write_json)
from .sim_link import SimLinkClient
from .visual import open_viewer

CTRL_DT = 1.0 / CONTROL_HZ


def _say(msg=""):
    print(msg, flush=True)


def _prompt(text):
    _say()
    input(f">>> {text} ")


# ----------------------------------------------------------------------------- run
def run(cfg):
    cfg = {k: v for k, v in cfg.items() if k.isupper()}
    gait = cfg["GAIT_NAME"]
    sim2real = int(cfg.get("SIM_2_REAL", 0))
    if sim2real not in (0, 1):
        raise ValueError("SIM_2_REAL must be 0 or 1")
    if float(cfg.get("LOOP_HZ", CONTROL_HZ)) != CONTROL_HZ:
        raise ValueError("The simulation controller runs at 200 Hz, like the hardware")
    _say("=" * 78)
    _say(f"OSL V2 MuJoCo gait test: {gait}   SIM_2_REAL = {sim2real}   "
         f"({platform.system()} {platform.machine()}, Python {platform.python_version()})")
    _say("=" * 78)

    local_ref = build_reference(cfg)
    for note in local_ref.notes:
        _say("  " + note)
    gains = {"knee": (cfg["KNEE_KP"], cfg["KNEE_KI"], cfg["KNEE_KD"]),
             "ankle": (cfg["ANKLE_KP"], cfg["ANKLE_KI"], cfg["ANKLE_KD"])}
    ref = local_ref
    run_dir = make_run_dir(gait)
    _say(f"[run] Results folder: {run_dir}")
    info = dict(gait=gait, sim_2_real=sim2real, osl_sim_version=__version__,
                platform=platform.platform(), python=platform.python_version(),
                started_local=time.strftime("%Y-%m-%d %H:%M:%S"),
                settings={k: v for k, v in cfg.items() if not k.endswith("TOKEN")},
                local_reference_notes=local_ref.notes)
    status, link, viewer = "NOT_STARTED", None, None
    ctrl_rows, full_rows = [], []
    sim = None
    t_go = None
    try:
        initial = {j: ref.ref_sim(j, 0.0) for j in JOINTS}
        # ------------------------------------------------------------- connect + arm
        if sim2real:
            link = SimLinkClient(cfg["HARDWARE_HOST"], cfg.get("SIM_LINK_PORT", 8795),
                                 cfg.get("SIM_LINK_TOKEN", ""), gait=gait, log=_say,
                                 raw_log_path=run_dir / "hardware_raw.jsonl",
                                 link_timeout_s=cfg.get("LINK_TIMEOUT_S", 3.0))
            hw_info = link.connect()
            info["hardware_info"] = {k: v for k, v in hw_info.items() if k != "reference"}
            hw_ref = GaitReference.from_dict(hw_info["reference"])
            diff, ddur = hw_ref.max_difference_deg(local_ref)
            info["reference_difference_vs_local"] = dict(max_abs_deg=diff, duration_diff_s=ddur)
            _say(f"[sim-link] Leg reference vs this script's settings: max difference "
                 f"{diff:.2e} deg, duration difference {ddur:.2e} s")
            if diff > 1e-6 or ddur > 1e-9:
                _say("[sim-link] WARNING: the leg's settings differ from this script's "
                     "(STRIDE/TIME_SCALE/start/limits?).")
            hw_gains = {j: tuple(hw_info["joints"][j]["gains"]) for j in JOINTS}
            if hw_gains != {j: tuple(gains[j]) for j in JOINTS}:
                _say(f"[sim-link] NOTE: leg gains {hw_gains} differ from this script's {gains}.")
            if cfg.get("USE_HARDWARE_SETTINGS", True):
                ref, gains = hw_ref, hw_gains
                _say("[sim-link] Using the LEG's reference and gains (USE_HARDWARE_SETTINGS).")
            initial = {j: ref.ref_sim(j, 0.0) for j in JOINTS}
            if float(hw_info.get("loop_hz", CONTROL_HZ)) != CONTROL_HZ:
                raise RuntimeError("leg LOOP_HZ is not 200 Hz")
            enabled = [j for j in JOINTS if hw_info["joints"][j].get("enabled")]
            _say(f"[sim-link] Leg joints enabled: {', '.join(enabled) or 'NONE'}")
            _prompt("Press Enter to ARM the leg (it will power the motors and HOME to the "
                    "start pose), or Ctrl+C to quit:")
            link.arm()
            status = "ARMING"
            _say("[sim-link] ARM sent; waiting for the leg to home and settle...")
            ready = link.wait_event(("READY",), on_other=lambda m: _say(
                f"[leg] {m.get('phase', m['type'])}: {m.get('msg', '')}"))
            if ready["type"] != "READY":
                raise RuntimeError(f"leg stopped before READY: {ready.get('reason')} "
                                   f"{ready.get('status', '')}")
            for j, a in ready["angles_deg"].items():
                if a is not None:
                    initial[j] = SIM_FRAME_SIGN[j] * a
            info["leg_ready_angles_sim_frame_deg"] = initial
            _say(f"[sim-link] Leg READY at knee {initial['knee']:+.2f} deg, ankle "
                 f"{initial['ankle']:+.2f} deg (sim frame); simulation starts there.")

        # ------------------------------------------------------------- build the run
        n_ticks = int(math.floor(ref.duration_s * CONTROL_HZ + 1e-9)) + 1
        t_ticks = np.arange(n_ticks) * CTRL_DT
        refs = [{j: ref.ref_sim(j, t) for j in JOINTS} for t in t_ticks]
        write_csv(run_dir / "reference.csv",
                  [dict(time_s=t, knee_reference_deg=r["knee"], ankle_reference_deg=r["ankle"])
                   for t, r in zip(t_ticks, refs)])
        write_json(run_dir / "reference_knots.json", ref.to_dict())

        def collect(row):
            full_rows.append(row)

        sim = LegSimulation(initial, refs[0], gains=gains, on_step=collect)
        for j in JOINTS:                          # check every target BEFORE anything moves
            lo, hi = sim.joint_range_deg(j)
            vals = [r[j] for r in refs]
            if min(vals) < lo or max(vals) > hi:
                raise ValueError(f"{j} reference [{min(vals):.1f}, {max(vals):.1f}] deg is "
                                 f"outside the model's runtime range [{lo:.1f}, {hi:.1f}]")
        info["engine"] = sim.metadata()
        info["duration_s"] = ref.duration_s
        _say(f"[run] Trajectory: {ref.duration_s:.3f} s, {n_ticks} control ticks at 200 Hz; "
             f"gains knee {gains['knee']}, ankle {gains['ankle']}")
        if cfg.get("SHOW_VIEWER", True):
            viewer = open_viewer(sim.model, sim.data, log=_say)

        # ------------------------------------------------------------- start
        if sim2real:
            _prompt("Press Enter to START the leg AND the simulation together "
                    "(Ctrl+C = abort):")
            offset, rtt = link.sync_clocks()
            lead = float(cfg.get("GO_LEAD_S", 1.0))
            lead = max(lead, 3.0 * rtt + 0.05)
            t_go = link.go(offset, lead)
            info["clock_sync"] = dict(offset_s=offset, best_rtt_s=rtt,
                                      uncertainty_s=rtt / 2.0, go_lead_s=lead)
            _say(f"[sim-link] Clock offset {offset:+.6f} s (round trip {rtt * 1000:.1f} ms, "
                 f"start-time uncertainty <= {rtt * 500:.1f} ms). Starting in {lead:.2f} s...")
        else:
            if cfg.get("WAIT_FOR_ENTER_TO_START", True):
                _prompt("Press Enter to START the simulation:")
            t_go = time.perf_counter() + 0.5
        status = "RUNNING"

        # ------------------------------------------------------------- run loop
        realtime = bool(cfg.get("REALTIME", True)) or sim2real
        max_lag = 0.0
        hw_done_note = False
        while time.perf_counter() < t_go:
            time.sleep(min(0.002, max(0.0, t_go - time.perf_counter())))
        for i in range(n_ticks):
            if realtime:
                target = t_go + i * CTRL_DT
                while True:
                    now = time.perf_counter()
                    if now >= target:
                        break
                    time.sleep(min(0.001, target - now))
                lag = time.perf_counter() - target
                max_lag = max(max_lag, lag)
            else:
                lag = None
            before = sim.angles()
            k0 = len(full_rows)
            sim.control_tick(i, refs[i])
            row = {"tick": i, "time_s": i * CTRL_DT,
                   "realtime_lag_ms": None if lag is None else 1000.0 * lag}
            for j in JOINTS:
                first = next(r for r in full_rows[k0:] if r["joint"] == j)
                row.update({f"{j}_reference_deg": refs[i][j], f"{j}_sim_deg": before[j],
                            f"{j}_current_cmd_A": first["current_applied_A"],
                            f"{j}_current_request_A": first["current_requested_A"],
                            f"{j}_p_A": first["pid_p_A"], f"{j}_i_A": first["pid_i_A"],
                            f"{j}_d_A": first["pid_d_A"], f"{j}_clipped": first["current_clipped"],
                            f"{j}_joint_torque_Nm": first["joint_drive_torque_Nm"],
                            f"{j}_belt_deflection_rad": first["belt_deflection_rad"]})
            ctrl_rows.append(row)
            if viewer is not None:              # display thread draws it (never blocks)
                if viewer.is_running():
                    viewer.show(sim.data.qpos)
                else:
                    viewer = None
            if link is not None:
                if link.lost_reason:
                    raise RuntimeError(link.lost_reason)
                if link.done is not None and not hw_done_note:
                    hw_done_note = True
                    if link.done.get("status") != "COMPLETE":
                        _say(f"\n[leg] STOPPED EARLY: {link.done.get('reason')} -- the "
                             f"simulation finishes its own trajectory for the log.")
            if i % 200 == 0:
                _say(f"t={i * CTRL_DT:6.2f}s  knee ref {refs[i]['knee']:+7.2f} sim "
                     f"{before['knee']:+7.2f} | ankle ref {refs[i]['ankle']:+7.2f} sim "
                     f"{before['ankle']:+7.2f}" + (f" | leg samples {len(link.samples)}"
                                                   if link else ""))
        status = "COMPLETE"
        info["max_realtime_lag_s"] = max_lag
        _say(f"\n[run] Simulation finished (worst real-time lag {max_lag * 1000:.1f} ms).")
    except KeyboardInterrupt:
        status = "STOPPED_BY_USER"
        _say("\n[run] Ctrl+C -- stopping.")
    except SimulationDiverged as exc:
        status = f"SIM_DIVERGED: {exc}"
        _say(f"\n[run] {status}")
    except Exception as exc:                                    # noqa: BLE001
        status = f"STOPPED: {exc}"
        _say(f"\n[run] {status}")
        traceback.print_exc()
    finally:
        if link is not None:
            if status != "COMPLETE":
                link.stop(status)
            if link.done is None and link.lost_reason is None and t_go is not None:
                _say("[sim-link] Waiting for the leg to finish and send its last samples...")
                link.wait_event(("DONE",), timeout=10.0)
            info["leg_done"] = link.done
            info["leg_started"] = link.started
            info["link_lost_reason"] = link.lost_reason
            link.close()
        info["status"] = status
        _save(run_dir, info, ref, ctrl_rows, full_rows, link, sim)
    if viewer is not None and cfg.get("KEEP_VIEWER_OPEN", True):
        _say("[viewer] Close the MuJoCo window to exit.")
        try:
            while viewer.is_running():
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
    if viewer is not None:
        viewer.close()
    _say("\nDone.")
    return 0 if status == "COMPLETE" else 1


# ----------------------------------------------------------------------------- saving
def _save(run_dir, info, ref, ctrl_rows, full_rows, link, sim):
    try:
        write_csv(run_dir / "simulation.csv", ctrl_rows)
        steps = {}
        for r in full_rows:
            s = steps.setdefault(r["source_time_s"], {"time_s": r["source_time_s"]})
            j = r["joint"]
            s.update({f"{j}_sim_deg": r["sim_angle_deg"], f"{j}_target_deg": r["target_deg"],
                      f"{j}_current_cmd_A": r["current_applied_A"],
                      f"{j}_joint_torque_Nm": r["joint_drive_torque_Nm"],
                      f"{j}_actuator_torque_Nm": r["actuator_torque_Nm"],
                      f"{j}_friction_torque_Nm": r["friction_torque_Nm"],
                      f"{j}_belt_deflection_rad": r["belt_deflection_rad"],
                      f"{j}_joint_velocity_rad_s": r["joint_velocity_rad_s"]})
        full = list(steps.values())
        write_csv(run_dir / "simulation_full_rate.csv", full)
        hw = []
        if link is not None and link.samples:
            late = (link.started or {}).get("late_s") or 0.0
            info["hardware_start_late_s"] = late
            hw = hardware_rows(link.samples, time_shift_s=late)
            write_csv(run_dir / "hardware.csv", hw, HARDWARE_HEADER)
        summary = {"status": info.get("status")}
        if link is not None:
            done = info.get("leg_done") or {}
            summary["leg_status"] = done.get("status", "NO_DONE_RECEIVED")
            summary["leg_stop_reason"] = done.get("reason")
            summary["leg_samples_dropped"] = done.get("dropped")
            summary["start_alignment_uncertainty_s"] = (info.get("clock_sync") or {}).get(
                "uncertainty_s")
        if ctrl_rows and full:
            t0 = {j: ctrl_rows[0][f"{j}_sim_deg"] for j in JOINTS}
            sim_time = np.array([0.0] + [s["time_s"] for s in full])
            sim_angle = {j: np.array([t0[j]] + [s[f"{j}_sim_deg"] for s in full])
                         for j in JOINTS}
            pairs = pair_with_simulation(hw, sim_time, sim_angle) if hw else []
            write_csv(run_dir / "comparison.csv", pairs)
            summary["joints"] = summarize(pairs, ctrl_rows, sim_time, sim_angle)
            summary["hardware_ticks_received"] = len(hw)
            plot_comparison(run_dir / "comparison.png", ctrl_rows, hw,
                            f"{info['gait']} -- {'sim-to-real' if hw else 'simulation'} -- "
                            f"{info.get('status')}")
        info["summary"] = summary
        if sim is not None:
            info.setdefault("engine", sim.metadata())
        write_json(run_dir / "summary.json", summary)
        write_json(run_dir / "run_info.json", info)
        _say(f"[run] Saved run folder: {run_dir}")
        for j in JOINTS:
            s = summary.get("joints", {}).get(j, {})
            if s.get("simulation_tracking_rms_deg") is None:
                continue
            line = f"  {j:5s}: sim tracking RMS {s.get('simulation_tracking_rms_deg', float('nan')):.3f} deg"
            if s.get("sim_minus_hardware_rms_deg") is not None:
                line += (f" | hardware tracking RMS {s['hardware_tracking_rms_deg']:.3f} deg"
                         f" | sim-hardware RMS {s['sim_minus_hardware_rms_deg']:.3f} deg")
            _say(line)
        if summary.get("leg_status"):
            _say(f"  leg status: {summary['leg_status']}"
                 + (f" ({summary['leg_stop_reason']})" if summary.get("leg_stop_reason") else ""))
    except Exception:                                           # noqa: BLE001
        _say("[run] ERROR while saving results:")
        traceback.print_exc()
