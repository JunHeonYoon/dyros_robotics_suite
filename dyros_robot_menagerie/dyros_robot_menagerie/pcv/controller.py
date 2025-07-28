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

from dyros_robot_menagerie.pcv.robot_data import (
    PCVRobotData,
    TASK_DOF,
    WHEEL_DOF,
)
from dyros_robot_controller.robot_controller.mobile.base import MobileControllerBase

class PCVControllerPy(ControllerInterface):

    def __init__(self, node: Node, dt: float, mj_joint_dict: Dict[str, Any]):
        super().__init__(node, dt, mj_joint_dict)

        # robot data & low‑level controller
        self.robot_data   = PCVRobotData()
        self.controller   = MobileControllerBase(dt, self.robot_data)

        # ------------------------------------------------------------------ #
        # ROS 2 I/O                                                         #
        # ------------------------------------------------------------------ #
        self.key_sub = node.create_subscription(
            Int32, "pcv_controller/mode_input", self._key_cb, 10
        )
        self.pose_sub = node.create_subscription(
            PoseStamped, "pcv_controller/target_pose", self._pose_cb, 10
        )
        self.vel_sub = node.create_subscription(
            Twist, "pcv_controller/cmd_vel", self._vel_cb, 10
        )
        self.joint_sub = node.create_subscription(
            JointState, "/joint_states_raw", self._joint_state_cb, 10
        )

        self.pose_pub = node.create_publisher(
            PoseStamped, "pcv_controller/base_pose", 10
        )
        self.vel_pub = node.create_publisher(
            Twist, "pcv_controller/base_vel", 10
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
        self.wheel_pos[:] = np.array([pos_dict["front_left_steer"][0],
                                      pos_dict["front_left_rotate"][0],
                                      pos_dict["front_right_steer"][0],
                                      pos_dict["front_right_rotate"][0],
                                      pos_dict["rear_left_steer"][0],
                                      pos_dict["rear_left_rotate"][0],
                                      pos_dict["rear_right_steer"][0],
                                      pos_dict["rear_right_rotate"][0],
                                      ])
        self.wheel_vel[:] = np.array([vel_dict["front_left_steer"][0],
                                      vel_dict["front_left_rotate"][0],
                                      vel_dict["front_right_steer"][0],
                                      vel_dict["front_right_rotate"][0],
                                      vel_dict["rear_left_steer"][0],
                                      vel_dict["rear_left_rotate"][0],
                                      vel_dict["rear_right_steer"][0],
                                      vel_dict["rear_right_rotate"][0],])

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
            "front_left_steer":  float(self.wheel_vel_desired[0]),
            "front_left_rotate":  float(self.wheel_vel_desired[1]),
            "front_right_steer":  float(self.wheel_vel_desired[2]),
            "front_right_rotate":  float(self.wheel_vel_desired[3]),
            "rear_left_steer":  float(self.wheel_vel_desired[4]),
            "rear_left_rotate":  float(self.wheel_vel_desired[5]),
            "rear_right_steer":  float(self.wheel_vel_desired[6]),
            "rear_right_rotate":  float(self.wheel_vel_desired[7]),
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
