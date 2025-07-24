#pragma once
#include "mujoco_ros_sim/ControllerInterface.hpp"
#include "mujoco_ros_sim/ControllerRegistry.hpp"

#include "husky/robot_data.h"

#include "robot_controller/mobile/base.h"
#include "math_type_define.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace Husky
{
    /*
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

    */

    class HuskyController : public ControllerInterface
    {
        public:
            // ====================================================================================
            // ================================== Core Functions ================================== 
            // ====================================================================================
            HuskyController(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd);
            ~HuskyController() override;
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
            void subtargetVelCallback(const geometry_msgs::msg::Twist::SharedPtr);
            void subJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr);
            void pubBasePoseCallback();
            void pubBaseVelCallback();

            std::shared_ptr<HuskyRobotData> robot_data_;
            std::unique_ptr<RobotController::Mobile::MobileBase> robot_controller_;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       target_vel_sub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    joint_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    base_pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          base_vel_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       joint_pub_;

            rclcpp::TimerBase::SharedPtr base_pose_pub_timer_;
            rclcpp::TimerBase::SharedPtr base_vel_pub_timer_;

            bool is_mode_changed_{true};
            bool is_goal_pose_changed_{false};

            std::string mode_{"None"};
            
            double control_start_time_;
            double current_time_;

            //// Mobile Base pose wrt world frame
            TaskVec base_pose_;
            TaskVec base_pose_deisired_;
            TaskVec base_pose_init_;
            
            //// Mobile Base velocity wrt base frame
            TaskVec base_vel_;
            TaskVec base_vel_tmp;
            TaskVec base_vel_desired_;
            TaskVec base_vel_init_;

            WheelVec wheel_pos_;
            WheelVec wheel_vel_;

            //// control input
            WheelVec wheel_vel_desired_; // (left, right)
    };
} // namespace Husky