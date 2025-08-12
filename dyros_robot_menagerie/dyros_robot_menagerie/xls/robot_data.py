from __future__ import annotations
import numpy as np

from dyros_robot_controller import KinematicParam, DriveType
from dyros_robot_controller.robot_data import MobileBase

TASK_DOF  = 3
WHEEL_DOF = 4
"""
URDF Joint Information: XLS
 name                | value
---------------------+---------------------------
type                 | Mecanum
wheel_num            | 4
wheel_radius         | 0.1200

roller_angles (rad)
 idx | value
-----+------------
   0 |    -0.7854
   1 |     0.7854
   2 |     0.7854
   3 |    -0.7854

base2wheel_positions
 idx | position
-----+-------------------------
   0 | [0.2225  0.2045]
   1 | [ 0.2225  -0.2045]
   2 | [-0.2225   0.2045]
   3 | [-0.2225  -0.2045]

base2wheel_angles (rad)
 idx | value
-----+------------
   0 |     0.0000
   1 |     0.0000
   2 |     0.0000
   3 |     0.0000
"""
class XLSRobotData(MobileBase):

    TASK_DOF  = TASK_DOF
    WHEEL_DOF = WHEEL_DOF

    def __init__(self):
        param = KinematicParam(
            type                 = DriveType.Mecanum,
            wheel_radius         = 0.120,              # [m]
            base2wheel_positions = [np.array([0.2225, 0.2045]),
                                    np.array([0.2225, -0.2045]),
                                    np.array([-0.2225, 0.2045]),
                                    np.array([-0.2225, -0.2045])],
            base2wheel_angles    = [0,0,0,0],
            roller_angles        = [-np.pi/4, 
                                    np.pi/4,
                                    np.pi/4,
                                    -np.pi/4],
            max_lin_acc          = 3,
            max_ang_acc          = 6,
        )
        super().__init__(param)