import math
from typing import Dict, Any
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node   import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg   import JointState
from mujoco_ros_sim import ControllerInterface
from dyros_robot_menagerie.fr3_pcv.robot_data import (FR3PCVRobotData,
                                                      TASK_DOF,
                                                      VIRTUAL_DOF,
                                                      MANI_DOF,
                                                      MOBI_DOF,
                                                      JOINT_DOF
                                                      )
from dyros_robot_controller.robot_controller import MobileManipulatorControllerBase

"""
FR3 PCV MuJoCo Joint/Sensor Information
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  1 | front_left_steer     | _Hinge |  1 |  1 |     7 |    6
  2 | front_left_rotate    | _Hinge |  1 |  1 |     8 |    7
  3 | rear_left_steer      | _Hinge |  1 |  1 |     9 |    8
  4 | rear_left_rotate     | _Hinge |  1 |  1 |    10 |    9
  5 | rear_right_steer     | _Hinge |  1 |  1 |    11 |   10
  6 | rear_right_rotate    | _Hinge |  1 |  1 |    12 |   11
  7 | front_right_steer    | _Hinge |  1 |  1 |    13 |   12
  8 | front_right_rotate   | _Hinge |  1 |  1 |    14 |   13
  9 | fr3_joint1           | _Hinge |  1 |  1 |    15 |   14
 10 | fr3_joint2           | _Hinge |  1 |  1 |    16 |   15
 11 | fr3_joint3           | _Hinge |  1 |  1 |    17 |   16
 12 | fr3_joint4           | _Hinge |  1 |  1 |    18 |   17
 13 | fr3_joint5           | _Hinge |  1 |  1 |    19 |   18
 14 | fr3_joint6           | _Hinge |  1 |  1 |    20 |   19
 15 | fr3_joint7           | _Hinge |  1 |  1 |    21 |   20

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | front_left_rotate    | _Joint  | front_left_rotate
  1 | front_right_rotate   | _Joint  | front_right_rotate
  2 | rear_left_rotate     | _Joint  | rear_left_rotate
  3 | rear_right_rotate    | _Joint  | rear_right_rotate
  4 | front_left_steer     | _Joint  | front_left_steer
  5 | front_right_steer    | _Joint  | front_right_steer
  6 | rear_left_steer      | _Joint  | rear_left_steer
  7 | rear_right_steer     | _Joint  | rear_right_steer
  8 | fr3_joint1           | _Joint  | fr3_joint1
  9 | fr3_joint2           | _Joint  | fr3_joint2
 10 | fr3_joint3           | _Joint  | fr3_joint3
 11 | fr3_joint4           | _Joint  | fr3_joint4
 12 | fr3_joint5           | _Joint  | fr3_joint5
 13 | fr3_joint6           | _Joint  | fr3_joint6
 14 | fr3_joint7           | _Joint  | fr3_joint7

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:dyros_pcv_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:dyros_pcv_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:dyros_pcv_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:dyros_pcv_site


"""
class FR3PCVControllerPy(ControllerInterface):

    def __init__(self, node: Node, dt: float, mj_joint_dict: Dict[str, Any]) -> None:
        super().__init__(node, dt, mj_joint_dict)

        self.robot_data       = FR3PCVRobotData(verbose=True)
        self.robot_controller = MobileManipulatorControllerBase(dt, self.robot_data)

        ns = "fr3_pcv_controller"
        self._key_sub = node.create_subscription(Int32, f"{ns}/mode_input", self._key_cb, 10)
        self._target_pose_sub = node.create_subscription(PoseStamped, f"{ns}/target_pose", self._target_pose_cb, 10)
        self._target_base_vel_sub = node.create_subscription(Twist, f"{ns}/cmd_vel", self._vel_cb, 10)
        self._joint_sub = node.create_subscription(JointState, "/joint_states_raw", self._joint_cb, 10)

        self._ee_pose_pub = node.create_publisher(PoseStamped, f"{ns}/ee_pose", 10)
        self._joint_pub = node.create_publisher(JointState, "/joint_states", 10)
        
        self.mode: str             = "HOME"
        self.is_mode_changed       = True
        self.is_goal_pose_changed  = False
        
        self.current_time: float = 0.0
        self.control_start_time: float = 0.0

        # mobile base
        self.base_vel         = np.zeros(3)
        self.base_vel_desired = np.zeros(3)

        # joint states
        self.q_virtual            = np.zeros(VIRTUAL_DOF)
        self.q_virtual_init       = np.zeros(VIRTUAL_DOF)
        self.q_virtual_desired    = np.zeros(VIRTUAL_DOF)
        self.qdot_virtual         = np.zeros(VIRTUAL_DOF)
        self.qdot_virtual_init    = np.zeros(VIRTUAL_DOF)
        self.qdot_virtual_desired = np.zeros(VIRTUAL_DOF)
        
        self.q_mani            = np.zeros(MANI_DOF)
        self.q_mani_init       = np.zeros(MANI_DOF)
        self.q_mani_desired    = np.zeros(MANI_DOF)
        self.qdot_mani         = np.zeros(MANI_DOF)
        self.qdot_mani_init    = np.zeros(MANI_DOF)
        self.qdot_mani_desired = np.zeros(MANI_DOF)
        
        self.q_mobile            = np.zeros(MOBI_DOF)
        self.q_mobile_init       = np.zeros(MOBI_DOF)
        self.q_mobile_desired    = np.zeros(MOBI_DOF)
        self.qdot_mobile         = np.zeros(MOBI_DOF)
        self.qdot_mobile_init    = np.zeros(MOBI_DOF)

        # task‑space
        self.x            = np.eye(4)
        self.x_init       = np.eye(4)
        self.x_desired    = np.eye(4)
        self.xdot         = np.zeros(TASK_DOF)
        self.xdot_init    = np.zeros(TASK_DOF)
        self.xdot_desired = np.zeros(TASK_DOF)
        
        self.x_goal = np.eye(4)

        # outputs
        self.torque_mani_desired = np.zeros(MANI_DOF)
        self.qdot_mobile_desired = np.zeros(MOBI_DOF)

    def starting(self) -> None:
        self.ee_timer = self.node.create_timer(0.1, self._pub_ee_pose_cb)

    def updateState(self,
                    pos_dict:   Dict[str, np.ndarray],
                    vel_dict:   Dict[str, np.ndarray],
                    tau_ext:    Dict[str, np.ndarray],
                    sensor_dict: Dict[str, np.ndarray],
                    current_time: float
                    ) -> None:
        self.current_time = current_time

        # get virtual joint
        self.q_virtual[:2] = sensor_dict["position_sensor"][:2]
        yaw = R.from_quat(sensor_dict["orientation_sensor"], scalar_first=True).as_euler('zyx', degrees=False)[0]
        self.q_virtual[2] = yaw

        self.qdot_virtual[:2] = sensor_dict["linear_velocity_sensor"][:2]
        self.qdot_virtual[2]  = sensor_dict["angular_velocity_sensor"][2]

        # get mobile base velocity
        Rot = np.array([[ math.cos(yaw),  math.sin(yaw)],
                        [-math.sin(yaw),  math.cos(yaw)]])
        self.base_vel[:2] = Rot @ self.qdot_virtual[:2]
        self.base_vel[2]  = self.qdot_virtual[2]

        # get manipulator joint
        for i in range(MANI_DOF):
            jname = f"fr3_joint{i+1}"
            self.q_mani[i]  = pos_dict[jname][0]
            self.qdot_mani[i] = vel_dict[jname][0]

        # get mobile wheel joint
        self.q_mobile[:]  = np.array([pos_dict["front_left_steer"][0],
                                      pos_dict["front_left_rotate"][0],
                                      pos_dict["front_right_steer"][0],
                                      pos_dict["front_right_rotate"][0],
                                      pos_dict["rear_left_steer"][0],
                                      pos_dict["rear_left_rotate"][0],
                                      pos_dict["rear_right_steer"][0],
                                      pos_dict["rear_right_rotate"][0]])
        self.qdot_mobile[:] = np.array([vel_dict["front_left_steer"][0],
                                        vel_dict["front_left_rotate"][0],
                                        vel_dict["front_right_steer"][0],
                                        vel_dict["front_right_rotate"][0],
                                        vel_dict["rear_left_steer"][0],
                                        vel_dict["rear_left_rotate"][0],
                                        vel_dict["rear_right_steer"][0],
                                        vel_dict["rear_right_rotate"][0]])
        
        # update internal robot model
        if not self.robot_data.update_state(self.q_virtual,    self.q_mobile,    self.q_mani,
                                            self.qdot_virtual, self.qdot_mobile, self.qdot_mani):
            self.node.get_logger().error("[FR3PCVRobotData] Failed to update robot state.")

        # get ee
        self.x     = self.robot_data.get_pose()
        self.xdot  = self.robot_data.get_velocity()

    def compute(self) -> None:
        if self.is_mode_changed:
            self.is_mode_changed = False
            self.control_start_time = self.current_time
            
            self.q_virtual_init          = self.q_virtual.copy()
            self.qdot_virtual_init       = self.qdot_virtual.copy()
            self.q_virtual_desired       = self.q_virtual_init.copy()
            self.qdot_virtual_desired[:] = 0.0
            self.q_mani_init             = self.q_mani.copy()
            self.qdot_mani_init          = self.qdot_mani.copy()
            self.q_mani_desired          = self.q_mani_init.copy()
            self.qdot_mani_desired[:]    = 0.0
            self.q_mobile_init           = self.q_mobile.copy()
            self.qdot_mobile_init        = self.qdot_mobile.copy()
            self.q_mobile_desired        = self.q_mobile_init.copy()
            self.qdot_mobile_desired[:]  = 0.0
            
            self.x_init          = self.x.copy()
            self.xdot_init       = self.xdot.copy()
            self.x_desired       = self.x_init.copy()
            self.xdot_desired[:] = 0.0
            
            self.x_goal = self.x_init.copy()

        if self.mode == "HOME":
            q_mani_target = np.array([0, 0, 0, -math.pi/2, 0, math.pi/2, math.pi/4])
            self.torque_mani_desired = self.robot_controller.move_manipulator_joint_torque_cubic(q_mani_target    = q_mani_target, 
                                                                                                 qdot_mani_target = np.zeros(MANI_DOF),
                                                                                                 q_mani_init      = self.q_mani_init, 
                                                                                                 qdot_mani_init   = self.qdot_mani_init,
                                                                                                 current_time     = self.current_time, 
                                                                                                 init_time        = self.control_start_time, 
                                                                                                 duration         = 4.0)
            self.qdot_mobile_desired[:] = 0.0

        elif self.mode in ("QPIK", "QPID"):
            if self.is_goal_pose_changed:
                self.is_goal_pose_changed = False
                self.control_start_time = self.current_time
                
                self.x_init  = self.x.copy()
                self.xdot_init = self.xdot.copy()
                
            if self.mode == "QPIK":
                self.qdot_mobile_desired, self.qdot_mani_desired = self.robot_controller.QPIK_cubic(x_target     = self.x_goal, 
                                                                                                    xdot_target  = np.zeros(TASK_DOF),
                                                                                                    x_init       = self.x_init, 
                                                                                                    xdot_init    = self.xdot_init,
                                                                                                    current_time = self.current_time, 
                                                                                                    init_time    = self.control_start_time, 
                                                                                                    duration     = 4.0,
                                                                                                    link_name    = self.robot_data.ee_name
                                                                                                    )
                self.q_mani_desired += self.dt * self.qdot_mani_desired
                self.torque_mani_desired = self.robot_controller.move_manipulator_joint_torque_step(q_mani_target    = self.q_mani_desired, 
                                                                                                    qdot_mani_target = self.qdot_mani_desired
                                                                                                    )

            elif self.mode == "QPID": # TODO: WTF
                
                qddot_mobile_desired, self.torque_mani_desired = self.robot_controller.QPID_cubic(x_target     = self.x_goal, 
                                                                                                  xdot_target  = np.zeros(TASK_DOF),
                                                                                                  x_init       = self.x_init, 
                                                                                                  xdot_init    = self.xdot_init,
                                                                                                  current_time = self.current_time, 
                                                                                                  init_time    = self.control_start_time, 
                                                                                                  duration     = 4.0,
                                                                                                  link_name    = self.robot_data.ee_name
                                                                                                  )
                # self.qdot_mobile_desired = self.qdot_mobile + self.dt * qddot_mobile_desired
                self.qdot_mobile_desired += self.dt * qddot_mobile_desired
                
        else:
            self.torque_mani_desired = self.robot_data.get_gravity()[self.robot_data.get_joint_index().mani_start:self.robot_data.get_joint_index().mani_start+MANI_DOF]
            self.qdot_mobile_desired[:] = 0.0

    def getCtrlInput(self) -> Dict[str, float]:
        out = {"front_left_steer":   float(self.qdot_mobile_desired[0]),
               "front_left_rotate":  float(self.qdot_mobile_desired[1]),
               "front_right_steer":  float(self.qdot_mobile_desired[2]),
               "front_right_rotate": float(self.qdot_mobile_desired[3]),
               "rear_left_steer":    float(self.qdot_mobile_desired[4]),
               "rear_left_rotate":   float(self.qdot_mobile_desired[5]),
               "rear_right_steer":   float(self.qdot_mobile_desired[6]),
               "rear_right_rotate":  float(self.qdot_mobile_desired[7])}
        for i in range(MANI_DOF):
            out[f"fr3_joint{i+1}"] = self.torque_mani_desired[i]
        return out

    def _set_mode(self, mode: str) -> None:
        self.mode = mode
        self.is_mode_changed = True
        self.node.get_logger().info(f"Mode changed: {mode}")

    def _key_cb(self, msg: Int32):
        mapping = {1: "HOME", 
                   2: "QPIK", 
                   3: "QPID",
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

    def _vel_cb(self, msg: Twist):
        self.base_vel_desired[:] = [msg.linear.x, msg.linear.y, msg.angular.z]

    def _joint_cb(self, msg: JointState):
        js = JointState()
        js.header   = msg.header
        js.name     = list(msg.name)
        js.position = list(msg.position)
        js.velocity = list(msg.velocity)
        js.effort   = list(msg.effort)

        js.name.extend(["v_x_joint", "v_y_joint", "v_t_joint"])
        js.position.extend(self.q_virtual.tolist())
        js.velocity.extend(self.qdot_virtual.tolist())
        js.effort.extend([0.0, 0.0, 0.0])

        self._joint_pub.publish(js)

    def _pub_ee_pose_cb(self) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.node.get_clock().now().to_msg()

        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.x[:3, 3]
        quat = R.from_matrix(self.x[:3, :3]).as_quat()  # x, y, z, w
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat
        self._ee_pose_pub.publish(msg)
