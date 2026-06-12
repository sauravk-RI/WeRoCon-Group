"""
noise_profiler.py  —  Run this BEFORE imu_matlab_r0.4.py
==========================================================
Raspberry Pi 5  |  MPU9250 via I2C

This script measures the real noise characteristics of your sensor
while it is mounted on the prosthetic leg.  After all tests finish,
it prints tuned constants that you paste into imu_matlab_r0.4.py.

Usage:
    python3 noise_profiler.py

Output files (created in same directory):
    baseline.csv            — raw data from standing still test
    heel_strike.csv         — raw data from stamp test
    fft_walk.csv            — raw data from walking test
    drift.csv               — raw data from gyro drift test
    tuned_constants.json    — paste these into imu_matlab_r0.4.py

IMPORTANT:
    Sensor must already be strapped to the leg when you run this.
    Do NOT run it on a table — the results will be wrong for leg use.
"""

import time, math, csv, json, sys, struct, signal
import smbus2
import numpy as np

# ─── Shared register map (same as imu_matlab_r0.4.py) ────────────────
MPU_ADDR         = 0x68
AK_ADDR          = 0x0C
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_CFG2   = 0x1D
REG_INT_PIN_CFG  = 0x37
REG_USER_CTRL    = 0x6A
REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT   = 0x3B
REG_GYRO_XOUT    = 0x43
REG_WHO_AM_I     = 0x75

ACCEL_SCALE = 16384.0
GYRO_SCALE  = 65.5      # ±500 °/s — must match imu_matlab_r0.4.py


# ─── Minimal MPU9250 reader (no Kalman — raw data only) ───────────────
class RawMPU9250:
    def __init__(self):
        self.bus = smbus2.SMBus(1)
        self._setup()

    def _w(self, reg, val):
        self.bus.write_byte_data(MPU_ADDR, reg, val)

    def _r(self, reg, n):
        return bytes(self.bus.read_i2c_block_data(MPU_ADDR, reg, n))

    def _r1(self, reg):
        return self.bus.read_byte_data(MPU_ADDR, reg)

    def _setup(self):
        self._w(REG_PWR_MGMT_1, 0x00);  time.sleep(0.05)
        self._w(REG_PWR_MGMT_1, 0x01);  time.sleep(0.10)

        who = self._r1(REG_WHO_AM_I)
        print(f"  [MPU9250] WHO_AM_I = 0x{who:02X}")

        self._w(REG_SMPLRT_DIV,  0x00)
        self._w(REG_CONFIG,      0x04)   # DLPF 20 Hz
        self._w(REG_GYRO_CONFIG, 0x08)   # ±500 °/s
        self._w(REG_ACCEL_CONFIG,0x00)   # ±2 g
        self._w(REG_ACCEL_CFG2,  0x04)   # Accel DLPF 20 Hz
        self._w(REG_USER_CTRL,   0x00);  time.sleep(0.02)
        self._w(REG_INT_PIN_CFG, 0x02);  time.sleep(0.10)

    def read_all(self):
        """Returns ax, ay, az (g), gx, gy, gz (deg/s)"""
        a = struct.unpack('>hhh', self._r(REG_ACCEL_XOUT, 6))
        g = struct.unpack('>hhh', self._r(REG_GYRO_XOUT,  6))
        ax, ay, az = (v / ACCEL_SCALE for v in a)
        gx, gy, gz = (v / GYRO_SCALE  for v in g)
        return ax, ay, az, gx, gy, gz


# ─── Noise Profiler ───────────────────────────────────────────────────
class NoiseProfiler:
    def __init__(self, hz=100):
        print("\nInitialising sensor for profiling…")
        try:
            self.sensor = RawMPU9250()
        except Exception as e:
            print(f"[FATAL] Cannot init MPU9250: {e}")
            sys.exit(1)
        self.hz = hz
        self.dt = 1.0 / hz
        print("  Sensor ready.\n")

    # ── Test 1: Baseline noise floor ──────────────────────────────
    def run_baseline(self, duration=60):
        print("=" * 55)
        print(f"TEST 1 — BASELINE  ({duration} seconds)")
        print("  Strap sensor to leg.  Stand completely still.")
        print("  Starting in 5 seconds…")
        print("=" * 55)
        time.sleep(5)

        data    = self._record(duration, label="Baseline")
        pitches = [d["pitch_raw"] for d in data]
        rolls   = [d["roll_raw"]  for d in data]

        result = {
            "pitch_std_dev":   round(float(np.std(pitches)), 4),
            "pitch_peak_peak": round(float(max(pitches) - min(pitches)), 4),
            "roll_std_dev":    round(float(np.std(rolls)),   4),
            "mean_pitch":      round(float(np.mean(pitches)),4),
        }
        print(f"\n  Pitch noise floor  std dev  : {result['pitch_std_dev']}°")
        print(f"  Pitch noise floor  pk-pk    : {result['pitch_peak_peak']}°")
        print(f"  Roll  noise floor  std dev  : {result['roll_std_dev']}°")
        self._save_csv(data, "baseline.csv")
        return result

    # ── Test 2: Heel strike profile ───────────────────────────────
    def run_heel_strike(self, n_stamps=10):
        print("\n" + "=" * 55)
        print(f"TEST 2 — HEEL STRIKE  ({n_stamps} stamps)")
        print("  Stand still.  Every 5 seconds do ONE hard heel stamp.")
        print(f"  Total: {n_stamps} stamps × 5 s = {n_stamps*5} s")
        print("  Starting in 5 seconds…")
        print("=" * 55)
        time.sleep(5)

        data   = self._record(n_stamps * 5 + 5, label="Heel strike")
        events = []

        # Find spikes: accel_mag deviates > 0.3g from 1g
        for i, d in enumerate(data):
            if abs(d["accel_mag"] - 1.0) > 0.30:
                window = data[i : i + 25]   # 250 ms window
                if not window:
                    continue
                peak = max(abs(x["accel_mag"] - 1.0) for x in window)
                # Settling = first sample back within 0.08g of 1.0
                settle = next(
                    (j for j, x in enumerate(window)
                     if abs(x["accel_mag"] - 1.0) < 0.08),
                    len(window)
                )
                events.append({
                    "peak_g":        round(peak, 3),
                    "settle_samples": settle,
                    "settle_ms":     round(settle * self.dt * 1000, 1),
                })

        if not events:
            print("  WARNING: No heel strikes detected. Using safe defaults.")
            return {
                "n_detected":          0,
                "avg_peak_g":          0.5,
                "avg_settle_ms":       80.0,
                "recommended_hold_cycles": 8,
            }

        # De-duplicate — ignore spikes within 20 samples of each other
        deduped = [events[0]]
        for e in events[1:]:
            if e != deduped[-1]:
                deduped.append(e)
        events = deduped

        avg_peak    = float(np.mean([e["peak_g"]    for e in events]))
        avg_settle  = float(np.mean([e["settle_ms"] for e in events]))
        hold_cycles = math.ceil(avg_settle / (self.dt * 1000))

        result = {
            "n_detected":           len(events),
            "avg_peak_g":           round(avg_peak,   3),
            "avg_settle_ms":        round(avg_settle,  1),
            "recommended_hold_cycles": hold_cycles,
        }
        print(f"\n  Detected {result['n_detected']} heel strikes")
        print(f"  Avg spike magnitude : {result['avg_peak_g']} g")
        print(f"  Avg settling time   : {result['avg_settle_ms']} ms")
        print(f"  → Recommended R_HOLD_CYCLES = {result['recommended_hold_cycles']}")
        self._save_csv(data, "heel_strike.csv")
        return result

    # ── Test 3: Frequency analysis (FFT) ─────────────────────────
    def run_fft(self, duration=30):
        print("\n" + "=" * 55)
        print(f"TEST 3 — WALKING FFT  ({duration} seconds)")
        print("  Walk normally at your usual pace.")
        print("  Starting in 5 seconds…")
        print("=" * 55)
        time.sleep(5)

        data = self._record(duration, label="FFT walk")
        ax   = np.array([d["ax"] for d in data])
        ay   = np.array([d["ay"] for d in data])
        az   = np.array([d["az"] for d in data])

        # Use accel magnitude deviation — cleaner than single axis
        amag = np.sqrt(ax**2 + ay**2 + az**2) - 1.0
        freqs = np.fft.rfftfreq(len(amag), self.dt)
        power = np.abs(np.fft.rfft(amag)) ** 2

        # Noise band = above 10 Hz (gait signal is below 10 Hz)
        noise_mask        = freqs > 10.0
        signal_mask       = freqs <= 10.0
        noise_peak_freq   = float(freqs[noise_mask][np.argmax(power[noise_mask])])
        signal_power      = float(np.sum(power[signal_mask]))
        noise_power       = float(np.sum(power[noise_mask]))

        snr_db = 10.0 * math.log10(signal_power / noise_power) if noise_power > 0 else 99.0

        # DLPF recommendation
        if noise_peak_freq > 30:
            dlpf = "20Hz  (register 0x04)"
        elif noise_peak_freq > 15:
            dlpf = "10Hz  (register 0x05)"
        else:
            dlpf = "10Hz  (register 0x05) — noise very close to signal band"

        result = {
            "dominant_noise_hz": round(noise_peak_freq, 1),
            "snr_db":            round(snr_db, 1),
            "recommended_dlpf":  dlpf,
        }
        print(f"\n  Dominant noise frequency : {result['dominant_noise_hz']} Hz")
        print(f"  Signal-to-noise ratio    : {result['snr_db']} dB")
        print(f"  → Recommended DLPF = {result['recommended_dlpf']}")
        self._save_csv(data, "fft_walk.csv")
        return result

    # ── Test 4: Gyro drift ────────────────────────────────────────
    def run_gyro_drift(self, duration=60):
        print("\n" + "=" * 55)
        print(f"TEST 4 — GYRO DRIFT  ({duration} seconds)")
        print("  Keep the leg completely still.  Do not move at all.")
        print("  Starting in 5 seconds…")
        print("=" * 55)
        time.sleep(5)

        data   = self._record(duration, label="Gyro drift")
        gy_all = [d["gy"] for d in data]
        gx_all = [d["gx"] for d in data]
        gz_all = [d["gz"] for d in data]

        gy_mean  = float(np.mean(gy_all))
        gy_std   = float(np.std(gy_all))
        drift_total = abs(sum(gy_all) * self.dt)

        result = {
            "gy_mean_bias_dps":    round(gy_mean,  4),
            "gy_std_dps":          round(gy_std,   4),
            "gx_mean_bias_dps":    round(float(np.mean(gx_all)), 4),
            "gz_mean_bias_dps":    round(float(np.mean(gz_all)), 4),
            "total_drift_deg_1min": round(drift_total, 2),
            "recommended_Q_bias":  round(gy_std * 0.1, 6),
        }
        print(f"\n  Gyro Y  mean bias      : {result['gy_mean_bias_dps']} °/s")
        print(f"  Gyro Y  std dev        : {result['gy_std_dps']} °/s")
        print(f"  Integrated drift/1min  : {result['total_drift_deg_1min']} °")
        print(f"  → Recommended Q_bias = {result['recommended_Q_bias']}")
        self._save_csv(data, "drift.csv")
        return result

    # ── Run all tests and print tuned constants ───────────────────
    def run_all_and_tune(self):
        print("\n" + "=" * 55)
        print("  MPU9250 NOISE PROFILER")
        print("  Sensor must be strapped to the leg for all tests.")
        print("=" * 55)

        b   = self.run_baseline(duration=60)
        hs  = self.run_heel_strike(n_stamps=10)
        f   = self.run_fft(duration=30)
        gd  = self.run_gyro_drift(duration=60)

        # Compute tuned constants from measured data
        R_still   = round(b["pitch_std_dev"] ** 2, 6)
        R_moving  = round((b["pitch_std_dev"] * 12) ** 2, 4)
        blend_g   = round(max(0.08, hs["avg_peak_g"] * 0.20), 3)

        tuned = {
            "R_STILL":          R_still,
            "R_MOVING":         R_moving,
            "R_HOLD_CYCLES":    hs["recommended_hold_cycles"],
            "BLEND_G":          blend_g,
            "Q_BIAS":           gd["recommended_Q_bias"],
            "gyro_offset_gx":   gd["gx_mean_bias_dps"],
            "gyro_offset_gy":   gd["gy_mean_bias_dps"],
            "gyro_offset_gz":   gd["gz_mean_bias_dps"],
            "noise_floor_pitch_std": b["pitch_std_dev"],
            "dominant_noise_hz":     f["dominant_noise_hz"],
            "snr_db":                f["snr_db"],
            "recommended_dlpf":      f["recommended_dlpf"],
        }

        print("\n" + "=" * 55)
        print("  TUNED CONSTANTS — paste into imu_matlab_r0.4.py")
        print("=" * 55)
        print(f"  R_STILL        = {tuned['R_STILL']}")
        print(f"  R_MOVING       = {tuned['R_MOVING']}")
        print(f"  R_HOLD_CYCLES  = {tuned['R_HOLD_CYCLES']}")
        print(f"  BLEND_G        = {tuned['BLEND_G']}")
        print(f"  Q_BIAS         = {tuned['Q_BIAS']}")
        print(f"\n  Gyro offsets to paste into gyro_off dict:")
        print(f"    gx : {tuned['gyro_offset_gx']}")
        print(f"    gy : {tuned['gyro_offset_gy']}")
        print(f"    gz : {tuned['gyro_offset_gz']}")
        print(f"\n  Info only (no paste needed):")
        print(f"    Noise floor pitch std : {tuned['noise_floor_pitch_std']}°")
        print(f"    Dominant noise freq   : {tuned['dominant_noise_hz']} Hz")
        print(f"    SNR                   : {tuned['snr_db']} dB")
        print(f"    DLPF advice           : {tuned['recommended_dlpf']}")
        print("=" * 55)

        with open("tuned_constants.json", "w") as f_j:
            json.dump(tuned, f_j, indent=2)
        print("\nAlso saved to tuned_constants.json")
        return tuned

    # ── Internal helpers ──────────────────────────────────────────
    def _record(self, duration, label="Recording"):
        data = []
        t0   = time.time()
        n    = 0
        while time.time() - t0 < duration:
            t_loop = time.time()
            try:
                ax, ay, az, gx, gy, gz = self.sensor.read_all()
            except OSError:
                time.sleep(0.01)
                continue

            accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
            pitch_raw = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
            roll_raw  = math.degrees(math.atan2(ay, az))

            data.append({
                "t":         round(time.time() - t0, 4),
                "ax": round(ax, 5), "ay": round(ay, 5), "az": round(az, 5),
                "gx": round(gx, 5), "gy": round(gy, 5), "gz": round(gz, 5),
                "accel_mag": round(accel_mag, 5),
                "pitch_raw": round(pitch_raw, 4),
                "roll_raw":  round(roll_raw,  4),
            })
            n += 1

            # Progress bar
            elapsed = time.time() - t0
            pct = int((elapsed / duration) * 100)
            bar = "#" * (pct // 5) + "." * (20 - pct // 5)
            print(f"\r  [{bar}] {pct:3d}%  {label}", end='', flush=True)

            spare = self.dt - (time.time() - t_loop)
            if spare > 0:
                time.sleep(spare)

        print()   # newline after progress bar
        print(f"  Recorded {n} samples at ~{round(n/duration)} Hz")
        return data

    def _save_csv(self, data, fname):
        if not data:
            return
        with open(fname, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=data[0].keys())
            w.writeheader()
            w.writerows(data)
        print(f"  Saved {fname}  ({len(data)} rows)")


# ─── Entry point ──────────────────────────────────────────────────────
if __name__ == "__main__":
    profiler = NoiseProfiler(hz=100)
    profiler.run_all_and_tune()
