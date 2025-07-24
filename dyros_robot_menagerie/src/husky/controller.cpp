#include "husky/controller.h"

namespace Husky
{
    HuskyController::HuskyController(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd)
    : ControllerInterface(node, dt, std::move(jd))
    {
        robot_data_ = std::make_shared<HuskyRobotData>();
        robot_controller_ = std::make_unique<RobotController::Mobile::MobileBase>(dt_, robot_data_);

        key_sub_ = node_->create_subscription<std_msgs::msg::Int32>("husky_controller/mode_input", 10,std::bind(&HuskyController::keyCallback, this, std::placeholders::_1));
        target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("husky_controller/target_pose", 10,std::bind(&HuskyController::subtargetPoseCallback, this, std::placeholders::_1));
        target_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("husky_controller/cmd_vel", 10,std::bind(&HuskyController::subtargetVelCallback, this, std::placeholders::_1));
        joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states_raw", 10, std::bind(&HuskyController::subJointStatesCallback, this, std::placeholders::_1));
        
        base_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("husky_controller/base_pose", 10);
        base_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("husky_controller/base_vel", 10);
        joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        
        base_pose_.setZero();
        base_pose_deisired_.setZero();
        base_pose_init_.setZero();

        base_vel_.setZero();
        base_vel_tmp.setZero();
        base_vel_desired_.setZero();
        base_vel_init_.setZero();

        wheel_vel_desired_.setZero();
    }

    HuskyController::~HuskyController()
    {

    }

    void HuskyController::starting()
    {
        base_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HuskyController::pubBasePoseCallback, this));
        base_vel_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HuskyController::pubBaseVelCallback, this));
    }

    void HuskyController::updateState(const VecMap& pos_dict, 
                                      const VecMap& vel_dict,
                                      const VecMap& tau_ext_dict, 
                                      const VecMap& sensors_dict, 
                                      double current_time)
    {
        current_time_ = current_time;

        // get mobile wheel joint
        wheel_pos_(0) = pos_dict.at("front_left_wheel")(0);
        wheel_pos_(1) = pos_dict.at("front_right_wheel")(0);

        wheel_vel_(0) = vel_dict.at("front_left_wheel")(0);
        wheel_vel_(1) = vel_dict.at("front_right_wheel")(0);

        // get base pose
        base_pose_.head(2) = sensors_dict.at("position_sensor").head(2);
        Quaterniond quat(sensors_dict.at("orientation_sensor")(0),
                         sensors_dict.at("orientation_sensor")(1),
                         sensors_dict.at("orientation_sensor")(2),
                         sensors_dict.at("orientation_sensor")(3));
        Vector3d euler_rpy = DyrosMath::rot2Euler(quat.toRotationMatrix());
        base_pose_(2) = euler_rpy(2);

        // get base vel
        Matrix2d rot_base2world;
        rot_base2world << cos(base_pose_(2)), sin(base_pose_(2)),
                         -sin(base_pose_(2)), cos(base_pose_(2));

        base_vel_.head(2) = rot_base2world * sensors_dict.at("linear_velocity_sensor").head(2);
        base_vel_(2) = sensors_dict.at("angular_velocity_sensor")(2);

        base_vel_tmp.head(2) = sensors_dict.at("linear_velocity_sensor").head(2);
        base_vel_tmp(2) = sensors_dict.at("angular_velocity_sensor")(2);
        
        robot_data_->updateState(wheel_pos_, wheel_vel_);
    }

    void HuskyController::compute()
    {
        if(is_mode_changed_)
        {
            is_mode_changed_ = false;
            control_start_time_ = current_time_;

            base_pose_init_ = base_pose_;
            base_vel_init_ = base_vel_;
            base_pose_deisired_ = base_pose_init_;
            base_vel_init_.setZero();
        }
        
        if(mode_ == "stop")
        {
            wheel_vel_desired_.setZero();
        }
        else if(mode_ == "base_vel_tracking")
        {
            wheel_vel_desired_ = robot_controller_->VelocityCommand(base_vel_desired_);
        }
        else
        {
            wheel_vel_desired_.setZero();
        }
    }


    CtrlInputMap HuskyController::getCtrlInput() const
    {
        CtrlInputMap ctrl_dict;
        ctrl_dict["left_wheel"] = wheel_vel_desired_(0);
        ctrl_dict["right_wheel"] = wheel_vel_desired_(1);
        return ctrl_dict;
    }

     void HuskyController::setMode(const std::string& mode)
    {
        is_mode_changed_ = true;
        mode_ = mode;
        RCLCPP_INFO(node_->get_logger(), "\033[34m Mode changed: %s\033[0m", mode.c_str());
    }

    void HuskyController::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Key input received: %d", msg->data);
        if(msg->data == 1)      setMode("stop");
        else if(msg->data == 2) setMode("base_vel_tracking");
        else                    setMode("none");
    }

    void HuskyController::subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Target pose received: position=(%.3f, %.3f, %.3f), "
                    "orientation=(%.3f, %.3f, %.3f, %.3f)",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                    msg->pose.orientation.x, msg->pose.orientation.y,
                    msg->pose.orientation.z, msg->pose.orientation.w);
        is_goal_pose_changed_ = true;

        // Convert to 4x4 homogeneous transform
        Eigen::Quaterniond quat(msg->pose.orientation.w,
                                msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z);

        Vector3d euler_rpy = DyrosMath::rot2Euler(quat.toRotationMatrix());
        base_pose_deisired_(2) = euler_rpy(2);
        base_pose_deisired_.head(2) << msg->pose.position.x,
                                       msg->pose.position.y;
    }

    void HuskyController::subtargetVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Target velocity received: linear=(%.3f, %.3f, %.3f), "
                    "angular=(%.3f, %.3f, %.3f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);

        base_vel_desired_.head(2) << msg->linear.x, msg->linear.y;
        base_vel_desired_(2) = msg->angular.z;
    }

    void HuskyController::subJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header = msg->header;
        joint_msg.name = msg->name;
        joint_msg.position = msg->position;
        joint_msg.velocity = msg->velocity;
        joint_msg.effort = msg->effort;

        const std::vector<std::string> virtual_joints = {"v_x_joint", "v_y_joint", "v_t_joint"};
        std::vector<double> virtual_pos_vec(base_pose_.data(), base_pose_.data() + base_pose_.size());
        std::vector<double> virtual_vel_vec(base_vel_tmp.data(), base_vel_tmp.data() + base_vel_tmp.size());
        std::vector<double> virtual_eff_vec{0,0,0};

        joint_msg.name.insert(joint_msg.name.end(), virtual_joints.begin(), virtual_joints.end());
        joint_msg.position.insert(joint_msg.position.end(), virtual_pos_vec.begin(), virtual_pos_vec.end());
        joint_msg.velocity.insert(joint_msg.velocity.end(), virtual_vel_vec.begin(), virtual_vel_vec.end());
        joint_msg.effort.insert(joint_msg.effort.end(), virtual_eff_vec.begin(), virtual_eff_vec.end());

        joint_pub_->publish(joint_msg);
    }

    void HuskyController::pubBasePoseCallback()
    {
        auto base_pose_msg = geometry_msgs::msg::PoseStamped();
        base_pose_msg.header.frame_id = "world";
        base_pose_msg.header.stamp = node_->now();

        base_pose_msg.pose.position.x = base_pose_(0);
        base_pose_msg.pose.position.y = base_pose_(1);
        base_pose_msg.pose.position.z = 0;

        Eigen::Quaterniond quat(Eigen::AngleAxisd(base_pose_(2), Eigen::Vector3d::UnitZ()));
        base_pose_msg.pose.orientation.x = quat.x();
        base_pose_msg.pose.orientation.y = quat.y();
        base_pose_msg.pose.orientation.z = quat.z();
        base_pose_msg.pose.orientation.w = quat.w();
        
        base_pose_pub_->publish(base_pose_msg);
    }

    void HuskyController::pubBaseVelCallback()
    {
        auto base_vel_msg = geometry_msgs::msg::Twist();
        base_vel_msg.linear.x = base_vel_(0);
        base_vel_msg.linear.y = base_vel_(1);
        base_vel_msg.angular.z = base_vel_(2);
        
        base_vel_pub_->publish(base_vel_msg);
    }

    /* register with the global registry */
    REGISTER_MJ_CONTROLLER(HuskyController, "HuskyController")
}