"""Every folder the simulation uses, resolved relative to this file.

Nothing here is an absolute path, so the whole Simulation_Codes folder can be
copied anywhere (another Linux machine, a Raspberry Pi, a Windows laptop) and
still work unchanged. pathlib handles the / vs \\ separator on each OS.
"""
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[1]          # .../Simulation_Codes
MODEL_PATH = SIM_ROOT / "models" / "osl_v2_bench_aluminium_2810.xml"
DATASETS_DIR = SIM_ROOT / "datasets"
RESULTS_DIR = SIM_ROOT / "results"
VALIDATION_DIR = SIM_ROOT / "validation"
