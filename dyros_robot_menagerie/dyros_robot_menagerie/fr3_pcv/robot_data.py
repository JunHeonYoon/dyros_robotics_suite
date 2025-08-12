from pathlib import Path
from typing import Final, Tuple
import numpy as np
from ament_index_python.packages import get_package_share_directory
from dyros_robot_controller import (DriveType,
                                    KinematicParam,
                                    JointIndex,
                                    ActuatorIndex,
                                    )
from dyros_robot_controller.robot_data import MobileManipulatorBase

TASK_DOF:     Final[int] = 6
VIRTUAL_DOF:  Final[int] = 3
MANI_DOF:     Final[int] = 7
MOBI_DOF:     Final[int] = 8
ACTUATOR_DOF: Final[int] = MANI_DOF + MOBI_DOF
JOINT_DOF:    Final[int] = ACTUATOR_DOF + VIRTUAL_DOF

"""
URDF Joint Information: FR3 PCV
Total nq = 18
Total nv = 18

 id | name                 | nq | nv | idx_q | idx_v
----+----------------------+----+----+-------+------
  1 |            v_x_joint |  1 |  1 |     0 |    0
  2 |            v_y_joint |  1 |  1 |     1 |    1
  3 |            v_t_joint |  1 |  1 |     2 |    2
  4 |           fr3_joint1 |  1 |  1 |     3 |    3
  5 |           fr3_joint2 |  1 |  1 |     4 |    4
  6 |           fr3_joint3 |  1 |  1 |     5 |    5
  7 |           fr3_joint4 |  1 |  1 |     6 |    6
  8 |           fr3_joint5 |  1 |  1 |     7 |    7
  9 |           fr3_joint6 |  1 |  1 |     8 |    8
 10 |           fr3_joint7 |  1 |  1 |     9 |    9
 11 |     front_left_steer |  1 |  1 |    10 |   10
 12 |    front_left_rotate |  1 |  1 |    11 |   11
 13 |    front_right_steer |  1 |  1 |    12 |   12
 14 |   front_right_rotate |  1 |  1 |    13 |   13
 15 |      rear_left_steer |  1 |  1 |    14 |   14
 16 |     rear_left_rotate |  1 |  1 |    15 |   15
 17 |     rear_right_steer |  1 |  1 |    16 |   16
 18 |    rear_right_rotate |  1 |  1 |    17 |   17

==================== Partition Indices ====================
 joint index
 name                | start
---------------------+------
 virtual             | 0
 manipulator         | 3
 mobile              | 10

 actuator index
 name                | start
---------------------+------
 manipulator         | 0
 mobile              | 7

======================= DoF Summary =======================
 total dof           | 18
 virtual dof         | 3
 mobile dof          | 8
 manipulator dof     | 7
 actuated dof        | 15

======================= Mobile Summary =======================
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
class FR3PCVRobotData(MobileManipulatorBase):
    def __init__(self, verbose: bool = False) -> None:
        robot_pkg = get_package_share_directory("dyros_robot_menagerie")
        mujoco_pkg = get_package_share_directory("mujoco_ros_sim")
        
        urdf  = str(Path(robot_pkg, "robot", "fr3_pcv.urdf"))
        srdf  = str(Path(robot_pkg, "robot", "fr3_pcv.srdf"))
        
        mobile_kine = KinematicParam(type         = DriveType.Caster,
                                     wheel_radius = 0.055,
                                     wheel_offset = 0.020,
                                     base2wheel_positions   = [np.array([ 0.215,  0.125]),
                                                               np.array([ 0.215, -0.125]),
                                                               np.array([-0.215,  0.125]),
                                                               np.array([-0.215, -0.125])],
                                     )
        joint_idx = JointIndex(virtual_start = 0,
                               mani_start    = VIRTUAL_DOF,
                               mobi_start    = VIRTUAL_DOF + MANI_DOF,
                               )
        actuator_idx = ActuatorIndex(mani_start = 0,
                                     mobi_start = MANI_DOF,
                                     )
        
        super().__init__(mobile_param  = mobile_kine,
                         urdf_path     = urdf,
                         srdf_path     = srdf,
                         packages_path = mujoco_pkg,
                         joint_idx     = joint_idx,
                         actuator_idx  = actuator_idx,
                         )
        
        self._ee_name: Final[str] = "fr3_link8"

    def compute_pose(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        return super().compute_pose(q_virtual, q_mobile, q_mani, self.ee_name)

    def compute_jacobian(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        return super().compute_jacobian(q_virtual, q_mobile, q_mani, self.ee_name)

    def compute_jacobian_time_variation(self,
                                        q_virtual: np.ndarray,
                                        q_mobile: np.ndarray,
                                        q_mani: np.ndarray,
                                        qdot_virtual: np.ndarray,
                                        qdot_mobile: np.ndarray,
                                        qdot_mani: np.ndarray,
    ) -> np.ndarray:
        return super().compute_jacobian_time_variation(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, self.ee_name)

    def compute_velocity(self,
                        q_virtual: np.ndarray,
                        q_mobile: np.ndarray,
                        q_mani: np.ndarray,
                        qdot_virtual: np.ndarray,
                        qdot_mobile: np.ndarray,
                        qdot_mani: np.ndarray,
    ) -> np.ndarray:
        return super().compute_velocity( q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, self.ee_name)

    def compute_jacobian_actuated(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        return super().compute_jacobian_actuated(q_virtual, q_mobile, q_mani, self.ee_name)

    def compute_jacobian_time_variation_actuated(self,
                                                 q_virtual: np.ndarray,
                                                 q_mobile: np.ndarray,
                                                 q_mani: np.ndarray,
                                                 qdot_virtual: np.ndarray,
                                                 qdot_mobile: np.ndarray,
                                                 qdot_mani: np.ndarray,
                                                 ) -> np.ndarray:
        return super().compute_jacobian_time_variation_actuated(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, self.ee_name)

    def compute_manipulability(self, q_mani: np.ndarray, qdot_mani: np.ndarray, with_grad: bool = False, with_graddot: bool = False) -> Tuple[float, np.ndarray, np.ndarray]:
        return super().compute_manipulability(q_mani, qdot_mani, with_grad, with_graddot, self.ee_name)

    def get_pose(self) -> np.ndarray:
        return np.asarray(super().get_pose(self.ee_name))

    def get_jacobian(self) -> np.ndarray:
        return np.asarray(super().get_jacobian(self.ee_name))

    def get_jacobian_time_variation(self) -> np.ndarray:
        return np.asarray(super().get_jacobian_time_variation(self.ee_name))

    def get_velocity(self) -> np.ndarray:
        return np.asarray(super().get_velocity(self.ee_name))

    def get_manipulability(self, with_grad: bool = False, with_graddot: bool = False)-> Tuple[float, np.ndarray, np.ndarray]:
        return super().get_manipulability(with_grad, with_graddot, self.ee_name)

    def get_jacobian_actuated(self) -> np.ndarray:
        return np.asarray(super().get_jacobian_actuated(self.ee_name))

    def get_jacobian_actuated_time_variation(self) -> np.ndarray:
        return np.asarray(super().get_jacobian_actuated_time_variation(self.ee_name))
    
    @property
    def ee_name(self) -> str:
        return self._ee_name