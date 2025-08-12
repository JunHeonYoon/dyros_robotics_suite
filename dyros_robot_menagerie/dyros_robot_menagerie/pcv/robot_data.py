from __future__ import annotations
import numpy as np

from dyros_robot_controller import KinematicParam, DriveType
from dyros_robot_controller.robot_data import MobileBase

TASK_DOF  = 3
WHEEL_DOF = 8

"""
URDF Joint Information: PCV
 name                | value
---------------------+---------------------------
type                 | Caster
wheel_num            | 8
wheel_radius         | 0.0550
offset               | 0.0200

base2wheel_positions
 idx | position
-----+-------------------------
   0 | [0.2150  0.1250]
   1 | [ 0.2150  -0.1250]
   2 | [-0.2150   0.1250]
   3 | [-0.2150  -0.1250]
"""

class PCVRobotData(MobileBase):

    TASK_DOF  = TASK_DOF
    WHEEL_DOF = WHEEL_DOF

    def __init__(self):
        param = KinematicParam(
            type                 = DriveType.Caster,
            wheel_radius         = 0.055,
            wheel_offset         = 0.020,
            base2wheel_positions = [np.array([0.215, 0.125]),
                                    np.array([0.215, -0.125]),
                                    np.array([-0.215, 0.125]),
                                    np.array([-0.215, -0.125])],
            max_lin_acc          = 3,
            max_ang_acc          = 6,
        )
        super().__init__(param)