"""Joint names and the hardware-encoder -> simulation angle convention (no MuJoCo needed).

sim frame (MuJoCo joint coordinates, and the hardware scripts' own CSV output):
    knee   positive = flexion         hardware encoder is flexion-NEGATIVE  -> x -1
    ankle  positive = plantarflexion  same sign as the hardware ankle encoder -> x +1
"""
JOINTS = ("knee", "ankle")
SIM_FRAME_SIGN = {"knee": -1.0, "ankle": 1.0}
