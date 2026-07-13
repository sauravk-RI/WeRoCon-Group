"""
imu_thigh_angle.py  —  MPU9250 Kalman Filter + CSV Logger for MATLAB
======================================================================
Raspberry Pi 5  |  MPU9250 via I2C  |  AK8963 magnetometer

Changes from r0.4:
  - Calibrated tuning constants pasted from noise_profiler output
  - DC_OFFSET_PITCH constant added  (from noise_profiler walking trial)
  - gyro_pitch_dps column added     (gy after offset subtraction — angular velocity)
  - pitch_centered_deg column added (pitch minus DC_OFFSET_PITCH — zero centered)
  - R_dynamic column added          (dynamic R per sample — diagnostic)
  - gyro_pitch_filt column added    (Kalman-filtered angular velocity — replaces EMA)
  - phase_var column added          (gait phase 0→1 via atan2 with fixed A=20, B=100)
  - All original equations, logic, and Kalman filter unchanged

Usage:
    pip install smbus2 numpy --break-system-packages
    python3 imu_matlab_r0.4.py

Output:
    imu_data.csv  (same format as r0.3 — MATLAB compatible)
    Auto-stops after RUN_DURATION seconds.
    Press Ctrl+C to stop early.

TUNING:
    Run noise_profiler.py first, then paste the printed values into
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
R_STILL        = 0.4371
R_MOVING       = 21.8550
R_HOLD_CYCLES  = 21
BLEND_G        = 0.060
Q_ANGLE        = 0.00003840
Q_BIAS         = 0.0000003840

# ─── Phase variable pre-processing ────────────────────────────────────
DC_OFFSET_PITCH = -3.587  # degrees — from noise_profiler walking trial

# ─── Phase variable normalization — fixed scaling from prior gait cycle ──
PHASE_A = 30.0    # max amplitude of pitch_centered_deg  (degrees)
PHASE_B = 130.0   # max amplitude of gyro_pitch_filt     (deg/s)

# ─── Gyro Kalman tuning ───────────────────────────────────────────────
GK_Q_OMEGA     = 0.25
GK_Q_OMEGADOT  = 1.0
GK_R           = 90.0

# ─── Phase variable monotonicity safeguard ────────────────────────────
MIN_INCREMENT     = 0.0005   # forced minimum forward step per loop
WRAP_THRESHOLD     = 0.85    # prev > this and new < (1-this) => real cycle wrap
MAX_CONSEC_CLAMP   = 50      # consecutive clamps before flagging sensor fault


# ─── Kalman Filter (with dynamic R and hold-off) ──────────────────────
class KalmanFilter:
    def __init__(self, Q_angle=Q_ANGLE, Q_bias=Q_BIAS, R_still=R_STILL):
        self.Q_angle = Q_angle
        self.Q_bias  = Q_bias
        self.R_still = R_still
        self.x = np.zeros((2, 1))
        self.P = np.eye(2)
        self.K = np.zeros((2, 1))
        self.H = np.array([[1.0, 0.0]])
        self._t = None

    def update(self, gyro_rate: float, meas_angle: float, R_dynamic: float) -> float:
        now = time.monotonic()
        if self._t is None:
            self._t = now
            self.x[0, 0] = meas_angle
            return meas_angle
        dt = now - self._t
        self._t = now
        F = np.array([[1.0, -dt], [0.0, 1.0]])
        B = np.array([[dt], [0.0]])
        Q = np.diag([self.Q_angle, self.Q_bias])
        self.x = F @ self.x + B * gyro_rate
        self.P = F @ self.P @ F.T + Q
        y = meas_angle - float((self.H @ self.x)[0, 0])
        S = float((self.H @ self.P @ self.H.T)[0, 0]) + R_dynamic
        self.K = (self.P @ self.H.T) / S
        self.x = self.x + self.K * y
        self.P = (np.eye(2) - self.K @ self.H) @ self.P
        return float(self.x[0, 0])

    @property
    def bias(self):
        return float(self.x[1, 0])


# ─── Gyro Kalman Filter ───────────────────────────────────────────────
class GyroKalman:
    def __init__(self, dt, q_omega, q_omegadot, r_meas):
        self.dt = dt
        self.omega     = 0.0
        self.omegadot  = 0.0
        self.P00 = 1.0;  self.P01 = 0.0
        self.P10 = 0.0;  self.P11 = 1.0
        self.q0  = q_omega
        self.q1  = q_omegadot
        self.R   = r_meas

    def update(self, omega_raw):
        dt = self.dt
        omega_pred    = self.omega    + self.omegadot * dt
        omegadot_pred = self.omegadot
        P00 = self.P00 + dt*(self.P10 + self.P01) + dt*dt*self.P11 + self.q0
        P01 = self.P01 + dt*self.P11
        P10 = self.P10 + dt*self.P11
        P11 = self.P11 + self.q1
        y  = omega_raw - omega_pred
        S  = P00 + self.R
        K0 = P00 / S
        K1 = P10 / S
        self.omega    = omega_pred + K0 * y
        self.omegadot = omegadot_pred + K1 * y
        self.P00 = (1 - K0) * P00
        self.P01 = (1 - K0) * P01
        self.P10 = P10 - K1 * P00
        self.P11 = P11 - K1 * P01
        return self.omega


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
        self._w(REG_PWR_MGMT_1, 0x00);  time.sleep(0.05)
        self._w(REG_PWR_MGMT_1, 0x01);  time.sleep(0.10)
        who = self._r1(REG_WHO_AM_I)
        print(f"[MPU9250] WHO_AM_I = 0x{who:02X}  (0x71=MPU9250  0x47=ICM-20689)")
        self._w(REG_SMPLRT_DIV,  0x00)
        self._w(REG_CONFIG,      0x05)
        self._w(REG_GYRO_CONFIG, 0x08)
        self._w(REG_ACCEL_CONFIG, 0x00)
        self._w(REG_ACCEL_CFG2,  0x05)
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
            if raw[6] & 0x08:
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
    accel_error = abs(accel_mag - 1.0)
    if accel_error > BLEND_G:
        return R_MOVING, R_HOLD_CYCLES
    if hold_counter > 0:
        return R_MOVING, hold_counter - 1
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

    gyro_off  = {'gx': -0.8146, 'gy': 1.9096, 'gz': -0.6619}
    accel_off = {'x':  0.0, 'y':  0.0, 'z':  0.0}
    accel_scl = {'x':  1.0, 'y':  1.0, 'z':  1.0}
    mag_hi    = {'x':  0.0, 'y':  0.0, 'z':  0.0}
    mag_si    = {'x':  1.0, 'y':  1.0, 'z':  1.0}

    gx_o, gy_o, gz_o = gyro_off['gx'], gyro_off['gy'], gyro_off['gz']
    mag_available = imu.mag_asa is not None

    # ── CSV setup ─────────────────────────────────────────────────
    csv_file = "phase_variable_mpu_9250.csv"
    f_csv    = open(csv_file, 'w')
    header = ("time_s,roll_deg,pitch_deg,yaw_deg,temp_c,bias_roll,bias_pitch,accel_norm_g,"
              "K_roll_angle,K_roll_bias,K_pitch_angle,K_pitch_bias,"
              "P_roll_00,P_roll_11,P_pitch_00,P_pitch_11,"
              "gyro_pitch_dps,pitch_centered_deg,R_dynamic,gyro_pitch_filt,"
              "phase_var\n")
    f_csv.write(header)
    f_csv.flush()

    print(f"\nLogging to : {csv_file}")
    print(f"Duration   : {RUN_DURATION} s   (Ctrl+C to stop early)")
    print(f"Gyro range : ±500 °/s   DLPF: 20 Hz   R_HOLD: {R_HOLD_CYCLES} cycles")
    print(f"Phase scaling : A={PHASE_A} deg   B={PHASE_B} deg/s\n")

    # ── Stabilization delay ───────────────────────────────────────
    STABILIZE_DURATION = 5.0
    print(f"Stabilizing for {STABILIZE_DURATION:.0f}s — keep IMU still…")
    t_stab_start = time.monotonic()
    while time.monotonic() - t_stab_start < STABILIZE_DURATION:
        try:
            ax, ay, az = imu.accel()
            gx, gy, gz = imu.gyro()
        except OSError:
            pass
        remaining = STABILIZE_DURATION - (time.monotonic() - t_stab_start)
        print(f"\r  {remaining:4.1f} s remaining…", end='', flush=True)
        time.sleep(1.0 / LOOP_HZ)
    print("\nStabilization done — starting logging.\n")

    # ── GyroKalman init ───────────────────────────────────────────
    dt_nominal = 1.0 / LOOP_HZ
    gyro_kf = GyroKalman(
        dt         = dt_nominal,
        q_omega    = GK_Q_OMEGA,
        q_omegadot = GK_Q_OMEGADOT,
        r_meas     = GK_R
    )

    running = [True]
    def _stop(sig, frame):
        running[0] = False
    signal.signal(signal.SIGINT, _stop)

    last_yaw      = 0.0
    dt_target     = 1.0 / LOOP_HZ
    t_start       = time.monotonic()
    row_count     = 0
    hold_roll     = 0
    hold_pitch    = 0
    gyro_pitch_filt = 0.0
    prev_pitch    = None
    t_prev        = None

    # ── Phase variable monotonicity safeguard state ────────────────
    prev_phase_var   = None
    clamp_count      = 0
    sensor_fault_flag = False

    while running[0]:
        t0    = time.monotonic()
        t_rel = t0 - t_start

        if t_rel >= RUN_DURATION:
            break

        try:
            ax, ay, az = imu.accel()
            gx, gy, gz = imu.gyro()
        except OSError:
            time.sleep(0.01)
            continue

        gx -= gx_o;  gy -= gy_o;  gz -= gz_o
        ax  = (ax - accel_off['x']) * accel_scl['x']
        ay  = (ay - accel_off['y']) * accel_scl['y']
        az  = (az - accel_off['z']) * accel_scl['z']

        accel_norm = math.sqrt(ax**2 + ay**2 + az**2)

        R_dyn, hold_roll  = compute_R_dynamic(accel_norm, hold_roll)
        _,     hold_pitch = compute_R_dynamic(accel_norm, hold_pitch)

        a_roll, a_pitch = accel_angles(ax, ay, az)

        roll  = kf_roll.update(gx,  a_roll,  R_dyn)
        pitch = kf_pitch.update(gy, a_pitch, R_dyn)

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

        pitch_centered  = pitch - DC_OFFSET_PITCH
        if prev_pitch is None or t_prev is None:
            gyro_pitch_dps = 0.0
        else:
            dt_actual      = t0 - t_prev
            gyro_pitch_dps = (pitch - prev_pitch) / dt_actual
        prev_pitch = pitch
        t_prev     = t0
        gyro_pitch_filt = gyro_kf.update(gyro_pitch_dps)

        # ── Phase variable — fixed A and B, atan2 → 0 to 1 ──────
        phi_raw  = math.atan2(gyro_pitch_filt / PHASE_B, pitch_centered / PHASE_A)  # -π to π
        phase_var = 1 - ((phi_raw + math.pi) / (2 * math.pi))                              #  0 to 1

        # ── Monotonicity safeguard ──────────────────────────────
        if prev_phase_var is None:
            clamp_count = 0
        else:
            is_wraparound = (prev_phase_var > WRAP_THRESHOLD) and (phase_var < (1 - WRAP_THRESHOLD))
            if is_wraparound:
                clamp_count = 0
            elif phase_var <= prev_phase_var:
                phase_var = prev_phase_var + MIN_INCREMENT
                clamp_count += 1
                if clamp_count > MAX_CONSEC_CLAMP:
                    sensor_fault_flag = True
            else:
                clamp_count = 0
        prev_phase_var = phase_var

        # ── Write CSV row ──────────────────────────────────────────
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
            f"{round(float(kf_pitch.P[1,1]), 6)},"
            f"{round(gyro_pitch_dps, 4)},"
            f"{round(pitch_centered, 4)},"
            f"{round(R_dyn,          4)},"
            f"{round(gyro_pitch_filt, 4)},"
            f"{round(phase_var,       4)}\n"
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
                f" | hold={hold_roll}"
                f" | φ={phase_var:.3f}"
                f" | fault={sensor_fault_flag}",
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
