#pragma once
#include "mujoco_ros_sim/ControllerInterface.hpp"

#include "fr3_xls/robot_data.h"

#include "robot_controller/mobile_manipulator/base.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace FR3XLS
{
/*
MuJoCo Model Information: fr3_xls
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
 53 | fr3_joint1           | _Hinge |  1 |  1 |    59 |   58
 54 | fr3_joint2           | _Hinge |  1 |  1 |    60 |   59
 55 | fr3_joint3           | _Hinge |  1 |  1 |    61 |   60
 56 | fr3_joint4           | _Hinge |  1 |  1 |    62 |   61
 57 | fr3_joint5           | _Hinge |  1 |  1 |    63 |   62
 58 | fr3_joint6           | _Hinge |  1 |  1 |    64 |   63
 59 | fr3_joint7           | _Hinge |  1 |  1 |    65 |   64

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | front_right_wheel    | _Joint  | front_right_wheel
  1 | front_left_wheel     | _Joint  | front_left_wheel
  2 | rear_right_wheel     | _Joint  | rear_right_wheel
  3 | rear_left_wheel      | _Joint  | rear_left_wheel
  4 | fr3_joint1           | _Joint  | fr3_joint1
  5 | fr3_joint2           | _Joint  | fr3_joint2
  6 | fr3_joint3           | _Joint  | fr3_joint3
  7 | fr3_joint4           | _Joint  | fr3_joint4
  8 | fr3_joint5           | _Joint  | fr3_joint5
  9 | fr3_joint6           | _Joint  | fr3_joint6
 10 | fr3_joint7           | _Joint  | fr3_joint7

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:xls_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:xls_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:xls_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:xls_site

 id | name                        | mode     | resolution
----+-----------------------------+----------+------------
  0 | d435_rgb                    | _Fixed   | 640x480
*/
   class FR3XLSController : public ControllerInterface
   {
        public:
            // ====================================================================================
            // ================================== Core Functions ================================== 
            // ====================================================================================
            FR3XLSController(const rclcpp::Node::SharedPtr& node);
            ~FR3XLSController() override;
            void starting() override;
            void updateState(const VecMap&, const VecMap&, const VecMap&, const VecMap&, double) override;
            void compute() override;
            CtrlInputMap getCtrlInput() const override;

            private:
                // ====================================================================================
                // ===================== Helper / CB / Background Thread Functions ==================== 
                // ====================================================================================
                void setMode(const std::string& mode);
                void keyCallback(const std_msgs::msg::Int32::SharedPtr);
                void subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
                void subtargetBaseVelCallback(const geometry_msgs::msg::Twist::SharedPtr);
                void subJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr);
                void pubEEPoseCallback();

                std::shared_ptr<FR3XLSRobotData> robot_data_;
                std::unique_ptr<RobotController::MobileManipulator::MobileManipulatorBase> robot_controller_;

                rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
                rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
                rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       base_vel_sub_;
                rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    joint_sub_;
                rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    ee_pose_pub_;
                rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       joint_pub_;

                rclcpp::TimerBase::SharedPtr ee_pose_pub_timer_;

                bool is_mode_changed_{true};
                bool is_goal_pose_changed_{false};

                std::string mode_{"HOME"};

                double control_start_time_;
                double current_time_;

                //// mobile base
                Vector3d base_vel_; // [lin_x, lin_y, ang] wrt base frame
                Vector3d base_vel_desired_;
                Vector3d base_vel_init_;
                
                //// joint space state
                VirtualVec q_virtual_;
                VirtualVec q_virtual_desired_;
                VirtualVec q_virtual_init_;
                VirtualVec qdot_virtual_;
                VirtualVec qdot_virtual_desired_;
                VirtualVec qdot_virtual_init_;

                ManiVec q_mani_;
                ManiVec q_mani_desired_;
                ManiVec q_mani_init_;
                ManiVec qdot_mani_;
                ManiVec qdot_mani_desired_;
                ManiVec qdot_mani_init_;

                MobiVec q_mobile_;
                MobiVec q_mobile_desired_;
                MobiVec q_mobile_init_;
                MobiVec qdot_mobile_;
                MobiVec qdot_mobile_init_;

                //// operation space state
                Affine3d x_;
                Affine3d x_desired_;
                Affine3d x_init_;
                TaskVec xdot_;
                TaskVec xdot_desired_;
                TaskVec xdot_init_;
                
                Affine3d x_goal_;

                //// control input
                ManiVec torque_mani_desired_;
                MobiVec qdot_mobile_desired_;
   };
} // namespace FR3XLS