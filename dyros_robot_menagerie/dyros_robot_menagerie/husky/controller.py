import math
from typing import Dict, Any
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from mujoco_ros_sim import ControllerInterface
from dyros_robot_menagerie.husky.robot_data import (HuskyRobotData,
                                                    TASK_DOF,
                                                    WHEEL_DOF,
                                                    )
from dyros_robot_controller.robot_controller import MobileControllerBase

"""
Husky MuJoCo Joint/Sensor Imformation
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  1 | front_left_wheel     | _Hinge |  1 |  1 |     7 |    6
  2 | front_right_wheel    | _Hinge |  1 |  1 |     8 |    7
  3 | rear_left_wheel      | _Hinge |  1 |  1 |     9 |    8
  4 | rear_right_wheel     | _Hinge |  1 |  1 |    10 |    9

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | left_wheel           | _Joint  | front_left_wheel
  1 | right_wheel          | _Joint  | front_right_wheel

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:husky_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:husky_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:husky_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:husky_site
"""
class HuskyControllerPy(ControllerInterface):

    def __init__(self, node: Node, dt: float, mj_joint_dict: Dict[str, Any]):
        super().__init__(node, dt, mj_joint_dict)

        self.robot_data       = HuskyRobotData()
        self.robot_controller = MobileControllerBase(dt, self.robot_data)

        ns = "husky_controller"
        self.key_sub = node.create_subscription(Int32, f"{ns}/mode_input", self._key_cb, 10)
        self.pose_sub = node.create_subscription(PoseStamped, f"{ns}/target_pose", self._pose_cb, 10)
        self.vel_sub = node.create_subscription(Twist, f"{ns}/cmd_vel", self._vel_cb, 10)
        self.joint_sub = node.create_subscription(JointState, "/joint_states_raw", self._joint_state_cb, 10)

        self.pose_pub = node.create_publisher(PoseStamped, f"{ns}/base_pose", 10)
        self.vel_pub = node.create_publisher(Twist, f"{ns}/base_vel", 10)
        self.joint_pub = node.create_publisher(JointState, "/joint_states", 10)

        self.mode: str = "HOME"
        self._mode_changed = True
        self._goal_pose_changed = False

        self.current_time: float = 0.0
        self.control_start_time: float = 0.0

        self.base_pose         = np.zeros(TASK_DOF)
        self.base_pose_desired = np.zeros(TASK_DOF)
        self.base_pose_init    = np.zeros(TASK_DOF)
        self.base_vel          = np.zeros(TASK_DOF)
        self.base_vel_init     = np.zeros(TASK_DOF)
        self.base_vel_desired  = np.zeros(TASK_DOF)

        self.base_vel_world    = np.zeros(TASK_DOF)
        
        self.wheel_pos         = np.zeros(WHEEL_DOF)
        self.wheel_vel         = np.zeros(WHEEL_DOF)
        
        self.wheel_vel_desired = np.zeros(WHEEL_DOF)

    def starting(self) -> None:
        self.pose_timer = self.node.create_timer(0.1, self._pub_pose_cb)
        self.vel_timer  = self.node.create_timer(0.1, self._pub_vel_cb)

    def updateState(self,
                   pos_dict: Dict[str, np.ndarray],
                   vel_dict: Dict[str, np.ndarray],
                   tau_ext_dict: Dict[str, np.ndarray],
                   sensor_dict: Dict[str, np.ndarray],
                   current_time: float,
                   ) -> None:
        self.current_time = current_time

        # get mobile wheel joint
        self.wheel_pos[:] = np.array([pos_dict["front_left_wheel"][0],
                                      pos_dict["front_right_wheel"][0]])
        self.wheel_vel[:] = np.array([vel_dict["front_left_wheel"][0],
                                      vel_dict["front_right_wheel"][0]])

        # get base pose (world)
        self.base_pose[:2] = sensor_dict["position_sensor"][:2]
        self.base_pose[2]  = R.from_quat(sensor_dict["orientation_sensor"], scalar_first=True).as_euler('zyx', degrees=False)[0]

        # base velocity  (world, base)
        self.base_vel_world[:2] = sensor_dict["linear_velocity_sensor"][:2]
        self.base_vel_world[2] = sensor_dict["angular_velocity_sensor"][2]
        rot = np.array([[ math.cos(self.base_pose[2]),  math.sin(self.base_pose[2])],
                        [-math.sin(self.base_pose[2]),  math.cos(self.base_pose[2])]])
        self.base_vel[:2] = rot @ self.base_vel_world[:2]
        self.base_vel[2]  = self.base_vel_world[2]

        # update internal robot model
        if not self.robot_data.update_state(self.wheel_pos, self.wheel_vel):
            self.node.get_logger().error("[HuskyRobotData] Failed to update robot state.")

    def compute(self) -> None:
        if self._mode_changed:
            self._mode_changed = False
            self.control_start_time = self.current_time
            
            self.base_pose_init      = self.base_pose.copy()
            self.base_pose_desired   = self.base_pose_init.copy()
            self.base_vel_init       = self.base_vel.copy()
            self.base_vel_desired[:] = 0.0

        if self.mode == "STOP":
            self.wheel_vel_desired[:] = 0.0
        elif self.mode == "Base Velocity Tracking":
            self.wheel_vel_desired = self.robot_controller.velocity_command(self.base_vel_desired)
        else:
            self.wheel_vel_desired[:] = 0.0

    def getCtrlInput(self) -> Dict[str, float]:
        return {"left_wheel":  self.wheel_vel_desired[0],
                "right_wheel": self.wheel_vel_desired[1],
                }

    def _set_mode(self, mode: str) -> None:
        self.mode = mode
        self._mode_changed = True
        self.node.get_logger().info(f"Mode changed: {mode}")

    def _key_cb(self, msg: Int32) -> None:
        mapping = {
            1: "STOP",
            2: "Base Velocity Tracking",
        }
        self._set_mode(mapping.get(msg.data, "NONE"))

    def _pose_cb(self, msg: PoseStamped):
        self.base_pose_desired[:2] = [msg.pose.position.x, msg.pose.position.y]
        self.base_pose_desired[2]  = R.from_quat(np.array([msg.pose.orientation.x,
                                                           msg.pose.orientation.y,
                                                           msg.pose.orientation.z,
                                                           msg.pose.orientation.w]), 
                                                 scalar_first=True).as_euler('zyx', degrees=False)[2]

    def _vel_cb(self, msg: Twist):
        self.base_vel_desired[:] = [msg.linear.x, msg.linear.y, msg.angular.z]

    def _joint_state_cb(self, msg: JointState):
        js = JointState()
        js.header   = msg.header
        js.name     = list(msg.name)
        js.position = list(msg.position)
        js.velocity = list(msg.velocity)
        js.effort   = list(msg.effort)

        js.name.extend(["v_x_joint", "v_y_joint", "v_t_joint"])
        js.position.extend(self.base_pose.tolist())
        js.velocity.extend(self.base_vel_world.tolist())
        js.effort.extend([0.0, 0.0, 0.0])

        self.joint_pub.publish(js)

    def _pub_pose_cb(self):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp    = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = self.base_pose[:2]
        qz = math.sin(self.base_pose[2] / 2.0)
        qw = math.cos(self.base_pose[2] / 2.0)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

    def _pub_vel_cb(self):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = self.base_vel
        self.vel_pub.publish(msg)
