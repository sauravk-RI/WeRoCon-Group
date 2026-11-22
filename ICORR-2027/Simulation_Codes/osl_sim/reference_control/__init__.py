"""Current-domain control + compliant drivetrain modules for the OSL V2 bench leg.

These five files are copied VERBATIM from the project's validated MuJoCo research
code (current.py, drivetrain.py, compliance.py, gait.py, feedback.py) so that the
simulation's control law, PID, current conversion and Best et al. belt/actuator model
are *identical* to the ones the published gait-replay results were produced with.

The ONLY edits are five import lines, changed from absolute
``from oslbench.control.X import ...`` to package-relative ``from .X import ...``
so that this package works on its own, without that project's whole ``oslbench`` tree:

    current.py   line 48   from .drivetrain import PAPER, DrivetrainParameters
    gait.py      line 44   from .current import ConversionAudit, ...
    gait.py      line 98   from .feedback import ActuatorDifferenceSource
    feedback.py  line 12   from .current import CurrentPID
    feedback.py  line 13   from .gait import GaitCurrentSource

Do not edit the physics/control code here. If the upstream modules are updated, copy
them again and re-apply only those import lines, then re-run ``validate_engine.py``.
"""
