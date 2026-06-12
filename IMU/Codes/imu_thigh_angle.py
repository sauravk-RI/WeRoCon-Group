"""
imu_thigh_angle  —  MPU9250 Kalman Filter + CSV Logger for MATLAB
======================================================================
Raspberry Pi 5  |  MPU9250 via I2C  |  AK8963 magnetometer
Usage:
    pip install smbus2 numpy --break-system-packages

Output:
    imu_data.csv  
    Auto-stops after RUN_DURATION seconds.
    Press Ctrl+C to stop early.

TUNING:
    Run tuning_constant.py first, then paste the printed values into
    the TUNING CONSTANTS section below.
"""

import math, struct, time, signal, sys
import smbus2
import numpy as np

# ─── Run settings ─────────────────────────────────────────────────────
LOOP_HZ      = 100
RUN_DURATION = 10.0     # seconds — change as needed

# ─── MPU9250 Register Map ─────────────────────────────────────────────
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
REG_TEMP_OUT     = 0x41
REG_WHO_AM_I     = 0x75

AK_WIA    = 0x00
AK_ST1    = 0x02
AK_XOUT_L = 0x03
AK_ST2    = 0x09
AK_CNTL1  = 0x0A
AK_ASAX   = 0x10

ACCEL_SCALE = 16384.0   # ±2 g  (unchanged)
GYRO_SCALE  = 65.5      # ±500 °/s  (was 131.0 for ±250 °/s)

# ─── TUNING CONSTANTS ─────────────────────────────────────────────────
# Default values below work reasonably for a leg-mounted MPU9250.
# For best results: run noise_profiler.py and paste its output here.
#
R_STILL        = 0.08     # Kalman R when sensor is still  (paste from profiler)
R_MOVING       = 2.5      # Kalman R during movement        (paste from profiler)
R_HOLD_CYCLES  = 12       # Samples to block accel after heel strike (paste from profiler)
BLEND_G        = 0.15     # Accel magnitude deviation (g) that triggers R_MOVING
Q_ANGLE        = 0.005    # Process noise — angle  (keep unless profiler says otherwise)
Q_BIAS         = 0.003    # Process noise — bias   (paste from profiler)


# ─── Kalman Filter (with dynamic R and hold-off) ──────────────────────
class KalmanFilter:
    """
    2-state Kalman filter:  x = [angle, gyro_bias]
    Now supports dynamic R (R changes based on how much the leg is moving)
    and a hold-off counter that blocks noisy accel right after heel strike.
    K and P matrices stored for CSV export to MATLAB.
    """
    def __init__(self, Q_angle=Q_ANGLE, Q_bias=Q_BIAS, R_still=R_STILL):
        self.Q_angle = Q_angle
        self.Q_bias  = Q_bias
        self.R_still = R_still

        self.x = np.zeros((2, 1))   # [angle, bias]
        self.P = np.eye(2)
        self.K = np.zeros((2, 1))   # stored for MATLAB export
        self.H = np.array([[1.0, 0.0]])
        self._t = None

    def update(self, gyro_rate: float, meas_angle: float,
               R_dynamic: float) -> float:
        """
        gyro_rate  : calibrated gyro in deg/s
        meas_angle : angle from accelerometer in degrees
        R_dynamic  : measurement noise — caller sets this based on accel_mag
        """
        now = time.monotonic()
        if self._t is None:
            self._t = now
            self.x[0, 0] = meas_angle
            return meas_angle
        dt = now - self._t
        self._t = now

        # Prediction
        F = np.array([[1.0, -dt], [0.0, 1.0]])
        B = np.array([[dt], [0.0]])
        Q = np.diag([self.Q_angle, self.Q_bias])
        self.x = F @ self.x + B * gyro_rate
        self.P = F @ self.P @ F.T + Q

        # Update with dynamic R
        y = meas_angle - float((self.H @ self.x)[0, 0])
        S = float((self.H @ self.P @ self.H.T)[0, 0]) + R_dynamic
        self.K = (self.P @ self.H.T) / S
        self.x = self.x + self.K * y
        self.P = (np.eye(2) - self.K @ self.H) @ self.P
        return float(self.x[0, 0])

    @property
    def bias(self):
        return float(self.x[1, 0])


# ─── MPU9250 Driver ───────────────────────────────────────────────────
class MPU9250:
    def __init__(self):
        self.bus     = smbus2.SMBus(1)
        self.mag_asa = (1.0, 1.0, 1.0)
        self._setup()

    def _w(self, reg, val, addr=MPU_ADDR):
        self.bus.write_byte_data(addr, reg, val)

    def _r(self, reg, n, addr=MPU_ADDR):
        return bytes(self.bus.read_i2c_block_data(addr, reg, n))

    def _r1(self, reg, addr=MPU_ADDR):
        return self.bus.read_byte_data(addr, reg)

    def _setup(self):
        # Wake up
        self._w(REG_PWR_MGMT_1, 0x00);  time.sleep(0.05)
        self._w(REG_PWR_MGMT_1, 0x01);  time.sleep(0.10)

        who = self._r1(REG_WHO_AM_I)
        print(f"[MPU9250] WHO_AM_I = 0x{who:02X}  (0x71=MPU9250  0x47=ICM-20689)")

        self._w(REG_SMPLRT_DIV,  0x00)  # Sample rate = gyro_rate / (1 + SMPLRT_DIV)

        # DLPF = 20 Hz  (0x04)
        # Was 0x03 (41 Hz) — now rejects socket vibration above 20 Hz
        self._w(REG_CONFIG,      0x04)

        # Gyro range = ±500 °/s  (0x08)
        # Was 0x00 (±250 °/s) — was clipping during fast leg swing
        self._w(REG_GYRO_CONFIG, 0x08)

        # Accel range = ±2 g  (unchanged)
        self._w(REG_ACCEL_CONFIG, 0x00)

        # Accel DLPF = 20 Hz  (0x04)
        # Was 0x03 (41 Hz)
        self._w(REG_ACCEL_CFG2,  0x04)

        # Enable bypass so Raspberry Pi can talk directly to AK8963
        self._w(REG_USER_CTRL,   0x00);  time.sleep(0.02)
        self._w(REG_INT_PIN_CFG, 0x02);  time.sleep(0.10)

        self._init_ak8963()

    def _init_ak8963(self):
        try:
            wia = self._r1(AK_WIA, AK_ADDR)
            print(f"[AK8963]  WHO_AM_I = 0x{wia:02X}  (expected 0x48)")
        except OSError:
            print("[AK8963]  WARNING: magnetometer not found at 0x0C — yaw = gyro only.")
            self.mag_asa = None
            return

        # Power down → Fuse ROM → read sensitivity → power down → continuous mode 2
        self._w(AK_CNTL1, 0x00, AK_ADDR);  time.sleep(0.02)
        self._w(AK_CNTL1, 0x0F, AK_ADDR);  time.sleep(0.02)
        asa = self._r(AK_ASAX, 3, AK_ADDR)
        self.mag_asa = tuple((v - 128) / 256.0 + 1.0 for v in asa)
        self._w(AK_CNTL1, 0x00, AK_ADDR);  time.sleep(0.02)
        self._w(AK_CNTL1, 0x16, AK_ADDR);  time.sleep(0.02)
        print(f"[AK8963]  Sensitivity adj = {tuple(round(a,4) for a in self.mag_asa)}")

    def accel(self):
        d = struct.unpack('>hhh', self._r(REG_ACCEL_XOUT, 6))
        return tuple(v / ACCEL_SCALE for v in d)

    def gyro(self):
        d = struct.unpack('>hhh', self._r(REG_GYRO_XOUT, 6))
        return tuple(v / GYRO_SCALE for v in d)

    def mag(self):
        if self.mag_asa is None:
            return None
        try:
            if not (self._r1(AK_ST1, AK_ADDR) & 0x01):
                return None
            raw = self._r(AK_XOUT_L, 7, AK_ADDR)
            if raw[6] & 0x08:   # overflow bit
                return None
            vals = struct.unpack('<hhh', raw[:6])
            return tuple(vals[i] * self.mag_asa[i] * 0.15 for i in range(3))
        except OSError:
            return None

    def temp(self):
        d = struct.unpack('>h', self._r(REG_TEMP_OUT, 2))[0]
        return d / 333.87 + 21.0


# ─── Angle helpers ────────────────────────────────────────────────────
def accel_angles(ax, ay, az):
    roll  = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
    return roll, pitch

def tilt_yaw(mx, my, mz, roll_r, pitch_r):
    cr, sr = math.cos(roll_r),  math.sin(roll_r)
    cp, sp = math.cos(pitch_r), math.sin(pitch_r)
    Bx =  mx*cp + my*sr*sp + mz*cr*sp
    By =          my*cr    - mz*sr
    y  = math.degrees(math.atan2(-By, Bx))
    return y + 360 if y < 0 else y

def compute_R_dynamic(accel_mag: float, hold_counter: int) -> tuple:
    """
    Returns (R_dynamic, new_hold_counter).

    Logic:
      1. If accel magnitude deviates from 1g by more than BLEND_G
         → spike detected (heel strike or fast motion)
         → lock R to R_MOVING and reset hold counter
      2. If hold counter is still counting down
         → keep R_MOVING (don't trust accel yet — ringing after impact)
      3. Otherwise
         → smoothly blend R between R_STILL and R_MOVING
    """
    accel_error = abs(accel_mag - 1.0)

    if accel_error > BLEND_G:
        # New spike — reset hold counter
        return R_MOVING, R_HOLD_CYCLES

    if hold_counter > 0:
        # Still within hold-off window after last spike
        return R_MOVING, hold_counter - 1

    # Normal blending: 0 deviation → R_STILL, full BLEND_G → R_MOVING
    blend = min(accel_error / BLEND_G, 1.0)
    R_dyn = R_STILL + blend * (R_MOVING - R_STILL)
    return R_dyn, 0


# ─── Main logger ──────────────────────────────────────────────────────
def main():
    print("Initialising MPU9250…")
    try:
        imu = MPU9250()
    except Exception as e:
        print(f"[FATAL] Could not init MPU9250: {e}")
        sys.exit(1)

    kf_roll  = KalmanFilter()
    kf_pitch = KalmanFilter()
    kf_yaw   = KalmanFilter()

    # ── Calibration offsets ───────────────────────────────────────
    # Paste values from tuning_constant.py  output here.
    # Until then, zeros = no correction applied.
    gyro_off  = {'gx': 0.0, 'gy': 0.0, 'gz': 0.0}
    accel_off = {'x':  0.0, 'y':  0.0, 'z':  0.0}
    accel_scl = {'x':  1.0, 'y':  1.0, 'z':  1.0}
    mag_hi    = {'x':  0.0, 'y':  0.0, 'z':  0.0}   # hard-iron offset
    mag_si    = {'x':  1.0, 'y':  1.0, 'z':  1.0}   # soft-iron scale

    gx_o, gy_o, gz_o = gyro_off['gx'], gyro_off['gy'], gyro_off['gz']
    mag_available = imu.mag_asa is not None

    # ── CSV setup ─────────────────────────────────────────────────
    csv_file = "imu_data.csv"
    f_csv    = open(csv_file, 'w')
    # Header is identical to r0.3 — MATLAB script needs no changes
    header = ("time_s,roll_deg,pitch_deg,yaw_deg,temp_c,bias_roll,bias_pitch,accel_norm_g,"
              "K_roll_angle,K_roll_bias,K_pitch_angle,K_pitch_bias,"
              "P_roll_00,P_roll_11,P_pitch_00,P_pitch_11\n")
    f_csv.write(header)
    f_csv.flush()

    print(f"\nLogging to : {csv_file}")
    print(f"Duration   : {RUN_DURATION} s   (Ctrl+C to stop early)")
    print(f"Gyro range : ±500 °/s   DLPF: 20 Hz   R_HOLD: {R_HOLD_CYCLES} cycles\n")

    running = [True]
    def _stop(sig, frame):
        running[0] = False
    signal.signal(signal.SIGINT, _stop)

    last_yaw      = 0.0
    dt_target     = 1.0 / LOOP_HZ
    t_start       = time.monotonic()
    row_count     = 0

    # Hold-off counters — one per Kalman axis that uses accel
    hold_roll  = 0
    hold_pitch = 0

    while running[0]:
        t0    = time.monotonic()
        t_rel = t0 - t_start

        if t_rel >= RUN_DURATION:
            break

        # ── Read sensor ───────────────────────────────────────────
        try:
            ax, ay, az = imu.accel()
            gx, gy, gz = imu.gyro()
        except OSError:
            time.sleep(0.01)
            continue

        # ── Apply calibration offsets ─────────────────────────────
        gx -= gx_o;  gy -= gy_o;  gz -= gz_o
        ax  = (ax - accel_off['x']) * accel_scl['x']
        ay  = (ay - accel_off['y']) * accel_scl['y']
        az  = (az - accel_off['z']) * accel_scl['z']

        # ── Accel magnitude — used for dynamic R ──────────────────
        accel_norm = math.sqrt(ax**2 + ay**2 + az**2)

        # ── Compute dynamic R for this sample ─────────────────────
        # Roll and pitch share the same accel_norm so same R logic
        R_dyn, hold_roll  = compute_R_dynamic(accel_norm, hold_roll)
        _,     hold_pitch = compute_R_dynamic(accel_norm, hold_pitch)

        # ── Accel angles ──────────────────────────────────────────
        a_roll, a_pitch = accel_angles(ax, ay, az)

        # ── Kalman update ─────────────────────────────────────────
        roll  = kf_roll.update(gx,  a_roll,  R_dyn)
        pitch = kf_pitch.update(gy, a_pitch, R_dyn)

        # ── Yaw — magnetometer if available, else gyro integrate ──
        if mag_available:
            m = imu.mag()
            if m:
                mx = (m[0] - mag_hi['x']) * mag_si['x']
                my = (m[1] - mag_hi['y']) * mag_si['y']
                mz = (m[2] - mag_hi['z']) * mag_si['z']
                mag_yaw  = tilt_yaw(mx, my, mz,
                                    math.radians(roll), math.radians(pitch))
                last_yaw = kf_yaw.update(gz, mag_yaw, R_STILL)
            else:
                last_yaw += gz * dt_target
                if last_yaw >  180: last_yaw -= 360
                if last_yaw < -180: last_yaw += 360
        else:
            last_yaw += gz * dt_target
            if last_yaw >  180: last_yaw -= 360
            if last_yaw < -180: last_yaw += 360

        # ── Write CSV row  ──────────────────
        f_csv.write(
            f"{round(t_rel,  4)},"
            f"{round(roll,   4)},"
            f"{round(pitch,  4)},"
            f"{round(last_yaw, 4)},"
            f"{round(imu.temp(), 2)},"
            f"{round(kf_roll.bias,  6)},"
            f"{round(kf_pitch.bias, 6)},"
            f"{round(accel_norm, 4)},"
            f"{round(float(kf_roll.K[0,0]),  6)},"
            f"{round(float(kf_roll.K[1,0]),  6)},"
            f"{round(float(kf_pitch.K[0,0]), 6)},"
            f"{round(float(kf_pitch.K[1,0]), 6)},"
            f"{round(float(kf_roll.P[0,0]),  6)},"
            f"{round(float(kf_roll.P[1,1]),  6)},"
            f"{round(float(kf_pitch.P[0,0]), 6)},"
            f"{round(float(kf_pitch.P[1,1]), 6)}\n"
        )
        row_count += 1

        if row_count % 100 == 0:
            f_csv.flush()

        if row_count % 10 == 0:
            print(
                f"\r  {int((t_rel/RUN_DURATION)*100):3d}%"
                f" | roll={roll:+7.2f}°"
                f" | pitch={pitch:+7.2f}°"
                f" | R={R_dyn:.3f}"
                f" | hold={hold_roll}",
                end='', flush=True
            )

        spare = dt_target - (time.monotonic() - t0)
        if spare > 0:
            time.sleep(spare)

    f_csv.flush()
    f_csv.close()
    print(f"\n\nSaved {row_count} rows → {csv_file}")


if __name__ == "__main__":
    main()
