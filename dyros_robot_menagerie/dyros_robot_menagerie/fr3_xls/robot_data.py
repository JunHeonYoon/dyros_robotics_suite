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
MOBI_DOF:     Final[int] = 4
ACTUATOR_DOF: Final[int] = MANI_DOF + MOBI_DOF
JOINT_DOF:    Final[int] = ACTUATOR_DOF + VIRTUAL_DOF

"""

"""
class FR3XLSRobotData(MobileManipulatorBase):
    def __init__(self) -> None:
        robot_pkg = get_package_share_directory("dyros_robot_menagerie")
        mujoco_pkg = get_package_share_directory("mujoco_ros_sim")
        
        urdf  = str(Path(robot_pkg, "robot", "fr3_xls.urdf"))
        srdf  = str(Path(robot_pkg, "robot", "fr3_xls.srdf"))
        
        mobile_kine = KinematicParam(type         = DriveType.Mecanum,
                                     wheel_radius = 0.120,
                                     base2wheel_positions = [np.array([ 0.2225,  0.2045]),
                                                             np.array([ 0.2225, -0.2045]),
                                                             np.array([-0.2225,  0.2045]),
                                                             np.array([-0.2225, -0.2045])],
                                       base2wheel_angles = [0,0,0,0],
                                       roller_angles = [-np.pi/4, 
                                                         np.pi/4,
                                                         np.pi/4,
                                                        -np.pi/4]
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