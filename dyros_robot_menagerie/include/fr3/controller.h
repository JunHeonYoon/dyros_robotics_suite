#pragma once
#include "mujoco_ros_sim/ControllerInterface.hpp"

#include "fr3/robot_data.h"

#include "robot_controller/manipulator/base.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace FR3
{
    /*
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
    */
    
    class FR3Controller : public ControllerInterface
    {
        public:
            // ====================================================================================
            // ================================== Core Functions ================================== 
            // ====================================================================================
            FR3Controller(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd);
            ~FR3Controller() override;
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
            void pubEEPoseCallback();

            std::shared_ptr<FR3RobotData> robot_data_;
            std::unique_ptr<RobotController::Manipulator::ManipulatorBase> robot_controller_;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    ee_pose_pub_;

            rclcpp::TimerBase::SharedPtr ee_pose_pub_timer_;
    
            bool is_mode_changed_{true};
            bool is_goal_pose_changed_{false};

            std::string mode_{"HOME"};
            
            double control_start_time_;
            double current_time_;

            //// joint space state
            JointVec q_;
            JointVec q_desired_;
            JointVec q_init_;
            JointVec qdot_;
            JointVec qdot_desired_;
            JointVec qdot_init_;
            
            //// operation space state
            Affine3d x_;
            Affine3d x_desired_;
            Affine3d x_init_;
            TaskVec xdot_;
            TaskVec xdot_desired_;
            TaskVec xdot_init_;
            
            Affine3d x_goal_;

            //// control input
            JointVec torque_desired_;
    };
} // namespace FR3Controller