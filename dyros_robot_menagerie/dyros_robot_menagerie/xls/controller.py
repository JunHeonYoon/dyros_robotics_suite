from __future__ import annotations

import math
import numpy as np
from typing import Dict, Any
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState

from mujoco_ros_sim import ControllerInterface

from dyros_robot_menagerie.xls.robot_data import (XLSRobotData,
                                                  TASK_DOF,
                                                  WHEEL_DOF,
                                                  )
from dyros_robot_controller.robot_controller.mobile.base import MobileControllerBase

"""
MuJoCo Model Information: summit_xls
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  1 | front_right_wheel    | _Hinge |  1 |  1 |     7 |    6
  2 | front_right_slipping_0_joint | _Hinge |  1 |  1 |     8 |    7
  3 | front_right_slipping_1_joint | _Hinge |  1 |  1 |     9 |    8
  4 | front_right_slipping_2_joint | _Hinge |  1 |  1 |    10 |    9
  5 | front_right_slipping_3_joint | _Hinge |  1 |  1 |    11 |   10
  6 | front_right_slipping_4_joint | _Hinge |  1 |  1 |    12 |   11
  7 | front_right_slipping_5_joint | _Hinge |  1 |  1 |    13 |   12
  8 | front_right_slipping_6_joint | _Hinge |  1 |  1 |    14 |   13
  9 | front_right_slipping_7_joint | _Hinge |  1 |  1 |    15 |   14
 10 | front_right_slipping_8_joint | _Hinge |  1 |  1 |    16 |   15
 11 | front_right_slipping_9_joint | _Hinge |  1 |  1 |    17 |   16
 12 | front_right_slipping_10_joint | _Hinge |  1 |  1 |    18 |   17
 13 | front_right_slipping_11_joint | _Hinge |  1 |  1 |    19 |   18
 14 | front_left_wheel     | _Hinge |  1 |  1 |    20 |   19
 15 | front_left_slipping_0_joint | _Hinge |  1 |  1 |    21 |   20
 16 | front_left_slipping_1_joint | _Hinge |  1 |  1 |    22 |   21
 17 | front_left_slipping_2_joint | _Hinge |  1 |  1 |    23 |   22
 18 | front_left_slipping_3_joint | _Hinge |  1 |  1 |    24 |   23
 19 | front_left_slipping_4_joint | _Hinge |  1 |  1 |    25 |   24
 20 | front_left_slipping_5_joint | _Hinge |  1 |  1 |    26 |   25
 21 | front_left_slipping_6_joint | _Hinge |  1 |  1 |    27 |   26
 22 | front_left_slipping_7_joint | _Hinge |  1 |  1 |    28 |   27
 23 | front_left_slipping_8_joint | _Hinge |  1 |  1 |    29 |   28
 24 | front_left_slipping_9_joint | _Hinge |  1 |  1 |    30 |   29
 25 | front_left_slipping_10_joint | _Hinge |  1 |  1 |    31 |   30
 26 | front_left_slipping_11_joint | _Hinge |  1 |  1 |    32 |   31
 27 | rear_right_wheel     | _Hinge |  1 |  1 |    33 |   32
 28 | rear_right_slipping_0_joint | _Hinge |  1 |  1 |    34 |   33
 29 | rear_right_slipping_1_joint | _Hinge |  1 |  1 |    35 |   34
 30 | rear_right_slipping_2_joint | _Hinge |  1 |  1 |    36 |   35
 31 | rear_right_slipping_3_joint | _Hinge |  1 |  1 |    37 |   36
 32 | rear_right_slipping_4_joint | _Hinge |  1 |  1 |    38 |   37
 33 | rear_right_slipping_5_joint | _Hinge |  1 |  1 |    39 |   38
 34 | rear_right_slipping_6_joint | _Hinge |  1 |  1 |    40 |   39
 35 | rear_right_slipping_7_joint | _Hinge |  1 |  1 |    41 |   40
 36 | rear_right_slipping_8_joint | _Hinge |  1 |  1 |    42 |   41
 37 | rear_right_slipping_9_joint | _Hinge |  1 |  1 |    43 |   42
 38 | rear_right_slipping_10_joint | _Hinge |  1 |  1 |    44 |   43
 39 | rear_right_slipping_11_joint | _Hinge |  1 |  1 |    45 |   44
 40 | rear_left_wheel      | _Hinge |  1 |  1 |    46 |   45
 41 | rear_left_slipping_0_joint | _Hinge |  1 |  1 |    47 |   46
 42 | rear_left_slipping_1_joint | _Hinge |  1 |  1 |    48 |   47
 43 | rear_left_slipping_2_joint | _Hinge |  1 |  1 |    49 |   48
 44 | rear_left_slipping_3_joint | _Hinge |  1 |  1 |    50 |   49
 45 | rear_left_slipping_4_joint | _Hinge |  1 |  1 |    51 |   50
 46 | rear_left_slipping_5_joint | _Hinge |  1 |  1 |    52 |   51
 47 | rear_left_slipping_6_joint | _Hinge |  1 |  1 |    53 |   52
 48 | rear_left_slipping_7_joint | _Hinge |  1 |  1 |    54 |   53
 49 | rear_left_slipping_8_joint | _Hinge |  1 |  1 |    55 |   54
 50 | rear_left_slipping_9_joint | _Hinge |  1 |  1 |    56 |   55
 51 | rear_left_slipping_10_joint | _Hinge |  1 |  1 |    57 |   56
 52 | rear_left_slipping_11_joint | _Hinge |  1 |  1 |    58 |   57

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | front_right_wheel    | _Joint  | front_right_wheel
  1 | front_left_wheel     | _Joint  | front_left_wheel
  2 | rear_right_wheel     | _Joint  | rear_right_wheel
  3 | rear_left_wheel      | _Joint  | rear_left_wheel

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:xls_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:xls_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:xls_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:xls_site

 id | name                        | mode     | resolution
----+-----------------------------+----------+------------
"""
class XLSControllerPy(ControllerInterface):

    def __init__(self, node: Node):
        super().__init__(node)
        
        self.dt = 0.01

        # robot data & low‑level controller
        self.robot_data   = XLSRobotData()
        self.controller   = MobileControllerBase(self.dt, self.robot_data)

        # ------------------------------------------------------------------ #
        # ROS 2 I/O                                                         #
        # ------------------------------------------------------------------ #
        self.key_sub = node.create_subscription(
            Int32, "xls_controller/mode_input", self._key_cb, 10
        )
        self.pose_sub = node.create_subscription(
            PoseStamped, "xls_controller/target_pose", self._pose_cb, 10
        )
        self.vel_sub = node.create_subscription(
            Twist, "xls_controller/cmd_vel", self._vel_cb, 10
        )
        self.joint_sub = node.create_subscription(
            JointState, "/joint_states_raw", self._joint_state_cb, 10
        )

        self.pose_pub = node.create_publisher(
            PoseStamped, "xls_controller/base_pose", 10
        )
        self.vel_pub = node.create_publisher(
            Twist, "xls_controller/base_vel", 10
        )
        self.pub_joint = node.create_publisher(
            JointState, "/joint_states", 10
        )

        self.pose_timer = node.create_timer(0.1, self._pub_pose_cb)
        self.vel_timer  = node.create_timer(0.1, self._pub_vel_cb)

        # states ------------------------------------------------------------ #
        self.mode                = "None"
        self.is_mode_changed     = True

        self.current_time        = 0.0
        self.base_pose           = np.zeros(TASK_DOF)
        self.base_pose_desired   = np.zeros(TASK_DOF)
        self.base_vel            = np.zeros(TASK_DOF)
        self.base_vel_desired    = np.zeros(TASK_DOF)

        self.wheel_pos           = np.zeros(WHEEL_DOF)
        self.wheel_vel           = np.zeros(WHEEL_DOF)
        self.wheel_vel_desired   = np.zeros(WHEEL_DOF)
        
        lines = [
            "=================================================================",
            "=================================================================",
            "URDF Joint Information: XLS",
            self.robot_data.get_verbose().rstrip("\n"),
            "=================================================================",
            "=================================================================",
        ]
        text = "\n".join(lines)
        self.node.get_logger().info("\033[1;34m\n" + text + "\033[0m")

    # ------------------------------------------------------------------ lifecycle
    def starting(self) -> None:
        # nothing else; publishers already active
        pass

    # ------------------------------------------------------------------ state update
    def updateState(
        self,
        pos_dict: Dict[str, Any],
        vel_dict: Dict[str, Any],
        tau_ext_dict: Dict[str, Any],
        sensor_dict: Dict[str, Any],
        current_time: float,
    ) -> None:
        self.current_time = current_time

        # wheel joints
        self.wheel_pos[:] = np.array([pos_dict["front_left_wheel"][0],
                                      pos_dict["front_right_wheel"][0],
                                      pos_dict["rear_left_wheel"][0],
                                      pos_dict["rear_right_wheel"][0]])
        self.wheel_vel[:] = np.array([vel_dict["front_left_wheel"][0],
                                      vel_dict["front_right_wheel"][0],
                                      vel_dict["rear_left_wheel"][0],
                                      vel_dict["rear_right_wheel"][0]])

        # base pose (world)
        self.base_pose[:2] = sensor_dict["position_sensor"][:2]
        self.base_pose[2]  = R.from_quat(sensor_dict["orientation_sensor"], scalar_first=True).as_euler('zyx', degrees=False)[0]

        # base velocity  (world → base)
        rot = np.array([[ math.cos(self.base_pose[2]),  math.sin(self.base_pose[2])],
                        [-math.sin(self.base_pose[2]),  math.cos(self.base_pose[2])]])
        lin_world = sensor_dict["linear_velocity_sensor"][:2]
        self.base_vel[:2] = rot @ lin_world
        self.base_vel[2]  = sensor_dict["angular_velocity_sensor"][2]

        # update internal robot model
        self.robot_data.update_state(self.wheel_pos, self.wheel_vel)

    # ------------------------------------------------------------------ compute
    def compute(self) -> None:
        if self.is_mode_changed:
            self.is_mode_changed = False

        if self.mode == "stop":
            self.wheel_vel_desired[:] = 0.0
        elif self.mode == "base_vel_tracking":
            self.wheel_vel_desired = self.controller.velocity_command(self.base_vel_desired)
        else:
            self.wheel_vel_desired[:] = 0.0

    # ------------------------------------------------------------------ output
    def getCtrlInput(self) -> Dict[str, float]:
        return {
            "front_left_wheel":  float(self.wheel_vel_desired[0]),
            "front_right_wheel": float(self.wheel_vel_desired[1]),
            "rear_left_wheel":  float(self.wheel_vel_desired[2]),
            "rear_right_wheel": float(self.wheel_vel_desired[3]),
        }

    # ================================ ROS 2 callbacks ===================== #
    def _set_mode(self, mode: str):
        self.is_mode_changed = True
        self.mode = mode
        self.node.get_logger().info(f"Mode changed: {mode}")

    def _key_cb(self, msg: Int32):
        if msg.data == 1:
            self._set_mode("stop")
        elif msg.data == 2:
            self._set_mode("base_vel_tracking")
        else:
            self._set_mode("None")

    def _pose_cb(self, msg: PoseStamped):
        self.base_pose_desired[:2] = [msg.pose.position.x, msg.pose.position.y]
        self.base_pose_desired[2]  = R.from_quat(
            np.array([msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z,
                      msg.pose.orientation.w]), scalar_first=True).as_euler('zyx', degrees=False)[2]

    def _vel_cb(self, msg: Twist):
        self.base_vel_desired[:] = [msg.linear.x, msg.linear.y, msg.angular.z]

    def _joint_state_cb(self, msg: JointState):
        # republish with virtual base joints
        js = JointState()
        js.header   = msg.header
        js.name     = list(msg.name)
        js.position = list(msg.position)
        js.velocity = list(msg.velocity)
        js.effort   = list(msg.effort)

        js.name.extend(["v_x_joint", "v_y_joint", "v_t_joint"])
        js.position.extend(self.base_pose.tolist())
        js.velocity.extend(self.base_vel.tolist())
        js.effort.extend([0.0, 0.0, 0.0])

        self.pub_joint.publish(js)

    # periodic pubs
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
