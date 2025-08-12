import numpy as np
from typing import Final
from dyros_robot_controller import KinematicParam, DriveType
from dyros_robot_controller.robot_data import MobileBase

TASK_DOF:  Final[int] = 3
WHEEL_DOF: Final[int] = 2
"""
URDF Joint Information: Husky
 name                | value
---------------------+---------------------------
type                 | Differential
wheel_num            | 2
wheel_radius         | 0.1651
base_width           | 1.0702
"""

class HuskyRobotData(MobileBase):
    def __init__(self):
        param = KinematicParam(type         = DriveType.Differential,
                               wheel_radius = 0.1651,
                               base_width   = 0.2854 * 2 * 1.875,
                               max_lin_acc  = 3,
                               max_ang_acc  = 6,
                               )
        super().__init__(param)