from __future__ import annotations
import numpy as np

from dyros_robot_controller import KinematicParam, DriveType
from dyros_robot_controller.robot_data import MobileBase

TASK_DOF  = 3
WHEEL_DOF = 8


class PCVRobotData(MobileBase):
    """PCV mobile-base geometry and state container."""

    TASK_DOF  = TASK_DOF
    WHEEL_DOF = WHEEL_DOF

    def __init__(self):
        param = KinematicParam(
            type         = DriveType.Caster,
            wheel_radius = 0.055,
            wheel_offset = 0.020,
            base2wheel_positions   = [np.array([0.215, 0.125]),
                                      np.array([0.215, -0.125]),
                                      np.array([-0.215, 0.125]),
                                      np.array([-0.215, -0.125])],
        )
        super().__init__(param)