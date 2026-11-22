#!/usr/bin/env python3
"""(copy of loadcell_read_test_osl.py + reset fix) Read the SRI M3564F load cell with the Open-Source-Leg NBLoadcellDAQ class (SPI1).

    pip install opensourceleg
    python3 loadcell_read_osl.py --tare   # no load: zero the sensor, then stream
    python3 loadcell_read_osl.py          # stream without taring

The OSL class keeps the offset only in memory, so it is re-measured on every run with --tare.
"""
import argparse
import collections
import time

import numpy as np
from opensourceleg.logging import LOGGER
from opensourceleg.sensors.loadcell import NBLoadcellDAQ

LOGGER._ensure_file_handler = lambda: None   # the library would otherwise create a .log file on the first log call

G = 9.80665
AVG_SAMPLES = 100
EXCITATION_V = 5.007                # measure J2 pin 1 to pin 14 and put the real value here
MATRIX = np.array([
    (-8.344189, -1912.95886, 31.095542, 5.691295, -6.311593, 1814.362687),
    (-38.275154, 1121.206361, -0.012994, -2127.946099, 12.118408, 1056.375086),
    (-984.814447, -11.163688, -962.72409, -23.826888, -949.723252, 4.760664),
    (19.884734, 0.16967, -0.459657, -0.155463, -19.741508, 0.18944),
    (-11.650934, -0.613806, 22.981705, 0.478966, -11.244318, 0.654913),
    (-0.178507, -26.357561, -0.167943, -26.511556, 0.246647, -26.257032)])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tare", action="store_true", help="zero the sensor first (no load!)")
    args = ap.parse_args()

    loadcell = NBLoadcellDAQ(
        calibration_matrix=MATRIX,
        excitation_voltage=EXCITATION_V,
        amp_gain=[34, 34, 34, 151, 151, 151],
        spi_bus=1, spi_cs=0,        # SPI1 -> /dev/spidev1.0 (the library default is bus 0!)
        data_rate=500,
        gains=[1] * 6,              # ADC programmable gain
    )
    
    last_kg_z = collections.deque(maxlen=AVG_SAMPLES)
    n, t0 = 0, time.perf_counter()
    loadcell.start()
    try:
        if args.tare:
            input("Make sure NOTHING is loading the sensor, then press Enter... ")
            loadcell.calibrate(1024)
        print("   t[s]     Fx[N]     Fy[N]     Fz[N]    Mx[Nm]    My[Nm]    Mz[Nm]   kg_x   kg_y   kg_z")
        while True:
            loadcell.update()
            w = np.array(loadcell.data)             # [Fx, Fy, Fz, Mx, My, Mz]
            kg = w[:3] / G
            n += 1
            last_kg_z.append(kg[2])
            print(f"{time.perf_counter() - t0:7.3f}  " + "  ".join(f"{v:8.2f}" for v in w)
                  + "  " + " ".join(f"{v:6.2f}" for v in kg))
    except KeyboardInterrupt:
        dt = time.perf_counter() - t0
        print(f"\nStopped. {n} samples in {dt:.1f} s = {n / dt:.0f} Hz")
        if last_kg_z:
            print(f"Average kg_z over the last {len(last_kg_z)} samples = {np.mean(last_kg_z):.3f} kg")
    finally:
        loadcell.stop()


if __name__ == "__main__":
    main()