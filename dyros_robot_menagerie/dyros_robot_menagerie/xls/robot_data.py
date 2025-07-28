from __future__ import annotations
import numpy as np

from dyros_robot_controller import KinematicParam, DriveType
from dyros_robot_controller.robot_data import MobileBase

TASK_DOF  = 3
WHEEL_DOF = 4


class XLSRobotData(MobileBase):
    """XLS mobile-base geometry and state container."""

    TASK_DOF  = TASK_DOF
    WHEEL_DOF = WHEEL_DOF

    def __init__(self):
        param = KinematicParam(
            type         = DriveType.Mecanum,
            wheel_radius = 0.120,              # [m]
            base2wheel_positions   = [np.array([0.2225, 0.2045]),
                                      np.array([0.2225, -0.2045]),
                                      np.array([-0.2225, 0.2045]),
                                      np.array([-0.2225, -0.2045])],
            base2wheel_angles = [0,0,0,0],
            roller_angles=[-np.pi/4, 
                                    np.pi/4,
                                    np.pi/4,
                                    -np.pi/4]
        )
        super().__init__(param)