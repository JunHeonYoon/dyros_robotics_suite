from pathlib import Path
from typing import Final, Tuple
import numpy as np
from ament_index_python.packages import get_package_share_directory
from dyros_robot_controller.robot_data.manipulator.base import ManipulatorBase

TASK_DOF:  Final[int] = 6
JOINT_DOF: Final[int] = 7

"""
URDF Joint Information: FR3
Total nq = 7
Total nv = 7

 id | name                 | nq | nv | idx_q | idx_v
----+----------------------+----+----+-------+------
  1 |           fr3_joint1 |  1 |  1 |     0 |    0
  2 |           fr3_joint2 |  1 |  1 |     1 |    1
  3 |           fr3_joint3 |  1 |  1 |     2 |    2
  4 |           fr3_joint4 |  1 |  1 |     3 |    3
  5 |           fr3_joint5 |  1 |  1 |     4 |    4
  6 |           fr3_joint6 |  1 |  1 |     5 |    5
  7 |           fr3_joint7 |  1 |  1 |     6 |    6
"""
class FR3RobotData(ManipulatorBase):
    def __init__(self) -> None:
        robot_pkg = get_package_share_directory("dyros_robot_menagerie")
        mujoco_pkg = get_package_share_directory("mujoco_ros_sim")

        urdf  = str(Path(robot_pkg, "robot", "fr3.urdf"))
        srdf  = str(Path(robot_pkg, "robot", "fr3.srdf"))

        super().__init__(urdf_path     = urdf,
                         srdf_path     = srdf,
                         packages_path = mujoco_pkg,
                         )

        self._ee_name: Final[str] = "fr3_link8"

    def compute_pose(self, q: np.ndarray) -> np.ndarray:
        return super().compute_pose(q, self._ee_name)

    def compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        return super().compute_jacobian(q, self._ee_name)

    def compute_jacobian_time_variation(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        return super().compute_jacobian_time_variation(q, qdot, self._ee_name)

    def compute_velocity(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        return super().compute_velocity(q, qdot, self._ee_name)

    def compute_manipulability(self, q: np.ndarray, qdot: np.ndarray, with_grad: bool = False, with_graddot: bool = False) -> Tuple[float, np.ndarray, np.ndarray]:
        return super().compute_manipulability(q, qdot, with_grad, with_graddot, self._ee_name)

    def get_pose(self) -> np.ndarray:
        return super().get_pose(self._ee_name)

    def get_jacobian(self) -> np.ndarray:
        return super().get_jacobian(self._ee_name)

    def get_jacobian_dot(self) -> np.ndarray:
        return super().get_jacobian_time_variation(self._ee_name)

    def get_velocity(self) -> np.ndarray:
        return super().get_velocity(self._ee_name)

    def get_manipulability(self, with_grad: bool = False, with_graddot: bool = False):
        return super().get_manipulability(with_grad, with_graddot, self._ee_name)
    
    @property
    def ee_name(self) -> str:
        return self._ee_name