"""osl_sim -- MuJoCo simulation + sim-to-real pipeline for the OSL V2 bench leg.

Package layout
    paths.py           folder locations, all relative to this package (portable)
    reference.py       gait reference builder, a faithful port of the hardware
                       scripts' dataset/stride/spline/auto-fit logic
    engine.py          LegSimulation -- the coupled knee+ankle MuJoCo plant, using the
                       final-paper drivetrain and Configuration C PID
    sim_link.py        TCP link shared with the leg Pi (identical copy lives in
                       Hardware_Codes/sim_link.py)
    runner.py          runs one gait test (standalone or sim-to-real), logs it
    results.py         run-folder writing, pairing, metrics, static plot
    reference_control/ control/drivetrain modules from the reference MuJoCo research
                       code (verbatim)
"""

__version__ = "1.0.0"
