import math
from typing import Dict, Any
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from mujoco_ros_sim import ControllerInterface
from dyros_robot_menagerie.fr3.robot_data import (FR3RobotData,
                                                  JOINT_DOF,
                                                  TASK_DOF,
                                                  )
from dyros_robot_controller.robot_controller import ManipulatorControllerBase

"""
FR3 MuJoCo Joint/Sensor Information
id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
0 | fr3_joint1           | _Hinge |  1 |  1 |     0 |    0
1 | fr3_joint2           | _Hinge |  1 |  1 |     1 |    1
2 | fr3_joint3           | _Hinge |  1 |  1 |     2 |    2
3 | fr3_joint4           | _Hinge |  1 |  1 |     3 |    3
4 | fr3_joint5           | _Hinge |  1 |  1 |     4 |    4
5 | fr3_joint6           | _Hinge |  1 |  1 |     5 |    5
6 | fr3_joint7           | _Hinge |  1 |  1 |     6 |    6

id | name                 | trn     | target_joint
----+----------------------+---------+-------------
0 | fr3_joint1           | _Joint  | fr3_joint1
1 | fr3_joint2           | _Joint  | fr3_joint2
2 | fr3_joint3           | _Joint  | fr3_joint3
3 | fr3_joint4           | _Joint  | fr3_joint4
4 | fr3_joint5           | _Joint  | fr3_joint5
5 | fr3_joint6           | _Joint  | fr3_joint6
6 | fr3_joint7           | _Joint  | fr3_joint7

id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
"""
class FR3ControllerPy(ControllerInterface):
    def __init__(self, node: Node, dt: float, mj_joint_dict: Dict[str, Any], ) -> None:
        super().__init__(node, dt, mj_joint_dict)

        self.robot_data       = FR3RobotData(verbose=True)
        self.robot_controller = ManipulatorControllerBase(dt, self.robot_data)
        
        ns = "fr3_controller"
        self._key_sub = node.create_subscription(Int32, f"{ns}/mode_input", self._key_cb, 10)
        self._target_pose_sub = node.create_subscription(PoseStamped, f"{ns}/target_pose", self._target_pose_cb, 10)
        
        self._ee_pose_pub = node.create_publisher(PoseStamped, f"{ns}/ee_pose", 10)
        
        self.mode: str = "NONE"
        self.is_mode_changed = True
        self.is_goal_pose_changed = False

        self.current_time: float = 0.0
        self.control_start_time: float = 0.0

        self.q            = np.zeros(JOINT_DOF)
        self.q_desired    = np.zeros(JOINT_DOF)
        self.q_init       = np.zeros(JOINT_DOF)
        self.qdot         = np.zeros(JOINT_DOF)
        self.qdot_desired = np.zeros(JOINT_DOF)
        self.qdot_init    = np.zeros(JOINT_DOF)

        self.x            = np.eye(4)
        self.x_init       = np.eye(4)
        self.x_desired    = np.eye(4)
        self.xdot         = np.zeros(TASK_DOF)
        self.xdot_init    = np.zeros(TASK_DOF)
        self.xdot_desired = np.zeros(TASK_DOF)

        self.x_goal = np.eye(4)

        self.torque_desired = np.zeros(JOINT_DOF)

    def starting(self) -> None:
        self._ee_timer = self.node.create_timer(0.1, self._pub_ee_pose_cb)

    def updateState(self,
                    pos_dict: Dict[str, np.ndarray],
                    vel_dict: Dict[str, np.ndarray],
                    tau_ext_dict: Dict[str, np.ndarray],
                    sensor_dict: Dict[str, np.ndarray],
                    current_time: float,
                    ) -> None:
        self.current_time = current_time

        # get manipulator joint
        for i in range(JOINT_DOF):
            name = f"fr3_joint{i + 1}"
            self.q[i]    = pos_dict[name][0]
            self.qdot[i] = vel_dict[name][0]

        if not self.robot_data.update_state(self.q, self.qdot):
            self.node.get_logger().error("[FR3RobotData] Failed to update robot state.")

        # get ee
        self.x     = self.robot_data.get_pose()
        self.x_dot = self.robot_data.get_velocity()

    def compute(self) -> None:
        if self.is_mode_changed:
            self.is_mode_changed = False
            self.control_start_time = self.current_time

            self.q_init          = self.q.copy()
            self.qdot_init       = self.qdot.copy()
            self.q_desired       = self.q_init.copy()
            self.qdot_desired[:] = 0.0

            self.x_init          = self.x.copy()
            self.xdot_init       = self.x_dot.copy()
            self.x_desired       = self.x_init.copy()
            self.xdot_desired[:] = 0.0
            
            self.x_goal  = self.x_init.copy()

        if self.mode == "HOME":
            target_q = np.array([0.0, 0.0, 0.0, -math.pi / 2.0, 0.0, math.pi / 2.0, math.pi / 4.0])
            self.torque_desired = self.robot_controller.move_joint_torque_cubic(q_target     = target_q,
                                                                                qdot_target  = np.zeros(JOINT_DOF),
                                                                                q_init       = self.q_init,
                                                                                qdot_init    = self.qdot_init,
                                                                                current_time = self.current_time,
                                                                                init_time    = self.control_start_time,
                                                                                duration     = 4.0,
                                                                                )

        elif self.mode in ("CLIK", "QPIK", "OSF", "QPID"):
            if self.is_goal_pose_changed:
                self.is_goal_pose_changed = False
                self.control_start_time = self.current_time
                
                self.x_init  = self.x.copy()
                self.xdot_init = self.x_dot.copy()

            if self.mode == "CLIK":
                self.qdot_desired = self.robot_controller.CLIK_cubic(x_target     = self.x_goal,
                                                                     xdot_target  = np.zeros(TASK_DOF),
                                                                     x_init       = self.x_init,
                                                                     xdot_init    = self.xdot_init,
                                                                     current_time = self.current_time,
                                                                     init_time    = self.control_start_time,
                                                                     duration     = 4.0,
                                                                     link_name    = self.robot_data.ee_name,
                                                                     )
                self.q_desired += self.dt * self.qdot_desired
                self.torque_desired = self.robot_controller.move_joint_torque_step(self.q_desired, 
                                                                                   self.qdot_desired)

            elif self.mode == "QPIK":
                self.qdot_desired = self.robot_controller.QPIK_cubic(x_target     = self.x_goal,
                                                                     xdot_target  = np.zeros(TASK_DOF),
                                                                     x_init       = self.x_init,
                                                                     xdot_init    = self.xdot_init,
                                                                     current_time = self.current_time,
                                                                     init_time    = self.control_start_time,
                                                                     duration     = 4.0,
                                                                     link_name    = self.robot_data.ee_name,
                                                                     )
                self.q_desired += self.dt * self.qdot_desired
                self.torque_desired = self.robot_controller.move_joint_torque_step(self.q_desired, 
                                                                                   self.qdot_desired)

            elif self.mode == "OSF":
                null_torque = self.robot_controller.move_joint_torque_step(self.q_init, 
                                                                           np.zeros(JOINT_DOF))
                self.torque_desired = self.robot_controller.OSF_cubic(x_target     = self.x_goal,
                                                                      xdot_target  = np.zeros(TASK_DOF),
                                                                      x_init       = self.x_init,
                                                                      xdot_init    = self.xdot_init,
                                                                      current_time = self.current_time,
                                                                      init_time    = self.control_start_time,
                                                                      duration     = 4.0,
                                                                      link_name    = self.robot_data.ee_name,
                                                                      null_torque  = null_torque
                                                                      )

            elif self.mode == "QPID":
                self.torque_desired = self.robot_controller.QPID_cubic(x_target     = self.x_goal,
                                                                      xdot_target  = np.zeros(TASK_DOF),
                                                                      x_init       = self.x_init,
                                                                      xdot_init    = self.xdot_init,
                                                                      current_time = self.current_time,
                                                                      init_time    = self.control_start_time,
                                                                      duration     = 4.0,
                                                                      link_name    = self.robot_data.ee_name,
                                                                      )

        else:
            self.torque_desired = self.robot_data.get_gravity()

    def getCtrlInput(self) -> Dict[str, float]:
        return {f"fr3_joint{i + 1}": float(self.torque_desired[i]) for i in range(JOINT_DOF)}

    def _set_mode(self, mode: str) -> None:
        self.mode = mode
        self.is_mode_changed = True
        self.node.get_logger().info(f"Mode changed: {mode}")

    def _key_cb(self, msg: Int32) -> None:
        mapping = {1: "HOME",
                   2: "CLIK",
                   3: "QPIK",
                   4: "OSF",
                   5: "QPID",
                   }
        self._set_mode(mapping.get(msg.data, "NONE"))

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        self.is_goal_pose_changed = True

        quat = np.array([msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z,
                         msg.pose.orientation.w,
                         ])
        rot = R.from_quat(quat).as_matrix()
        self.x_goal = np.eye(4)
        self.x_goal[:3, :3] = rot
        self.x_goal[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def _pub_ee_pose_cb(self) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.node.get_clock().now().to_msg()

        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.x[:3, 3]
        quat = R.from_matrix(self.x[:3, :3]).as_quat()  # x, y, z, w
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat
        self._ee_pose_pub.publish(msg)