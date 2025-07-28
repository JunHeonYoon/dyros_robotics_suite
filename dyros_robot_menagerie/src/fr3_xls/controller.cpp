#include "fr3_xls/controller.h"

namespace FR3XLS
{
    FR3XLSController::FR3XLSController(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd)
    : ControllerInterface(node, dt, std::move(jd))
    {
        robot_data_ = std::make_shared<FR3XLSRobotData>();
        robot_controller_ = std::make_unique<RobotController::MobileManipulator::MobileManipulatorBase>(dt_, robot_data_);

        key_sub_ = node_->create_subscription<std_msgs::msg::Int32>("fr3_xls_controller/mode_input", 10,std::bind(&FR3XLSController::keyCallback, this, std::placeholders::_1));
        target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("fr3_xls_controller/target_pose", 10,std::bind(&FR3XLSController::subtargetPoseCallback, this, std::placeholders::_1));
        base_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("fr3_xls_controller/cmd_vel", 10, std::bind(&FR3XLSController::subtargetBaseVelCallback, this, std::placeholders::_1));
        joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states_raw", 10, std::bind(&FR3XLSController::subJointStatesCallback, this, std::placeholders::_1));

        ee_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("fr3_xls_controller/ee_pose", 10);
        joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        base_vel_.setZero();
        base_vel_desired_.setZero();
        base_vel_init_.setZero();

        q_virtual_.setZero();
        q_virtual_desired_.setZero();
        q_virtual_init_.setZero();
        qdot_virtual_.setZero();
        qdot_virtual_desired_.setZero();
        qdot_virtual_init_.setZero();
        
        q_mani_.setZero();
        q_mani_desired_.setZero();
        q_mani_init_.setZero();
        qdot_mani_.setZero();
        qdot_mani_desired_.setZero();
        qdot_mani_init_.setZero();
        
        q_mobile_.setZero();
        q_mobile_desired_.setZero();
        q_mobile_init_.setZero();
        qdot_mobile_.setZero();
        qdot_mobile_init_.setZero();
        
        x_.setIdentity();
        x_desired_.setIdentity();
        x_init_.setIdentity();
        xdot_.setZero();
        xdot_desired_.setZero();
        xdot_init_.setZero();
        
        x_goal_.setIdentity();
        
        torque_mani_desired_.setZero();
        qdot_mobile_desired_.setZero();
    }

    FR3XLSController::~FR3XLSController()
    {

    }

    void FR3XLSController::starting()
    {
        ee_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FR3XLSController::pubEEPoseCallback, this));
    }

    void FR3XLSController::updateState(const VecMap& pos_dict, 
                                         const VecMap& vel_dict,
                                         const VecMap& tau_ext_dict, 
                                         const VecMap& sensors_dict, 
                                         double current_time)
    {
        current_time_ = current_time;

        
        // get virtual joint
        q_virtual_.head(2) = sensors_dict.at("position_sensor").head(2);
        Quaterniond quat(sensors_dict.at("orientation_sensor")(0),
                         sensors_dict.at("orientation_sensor")(1),
                         sensors_dict.at("orientation_sensor")(2),
                         sensors_dict.at("orientation_sensor")(3));
        Vector3d euler_rpy = DyrosMath::rot2Euler(quat.toRotationMatrix());
        q_virtual_(2) = euler_rpy(2);

        qdot_virtual_.head(2) = sensors_dict.at("linear_velocity_sensor").head(2);
        qdot_virtual_(2) = sensors_dict.at("angular_velocity_sensor")(2);
        
        // get mobile base velocity
        Matrix2d rot_base2world;
        rot_base2world << cos(q_virtual_(2)), sin(q_virtual_(2)),
                         -sin(q_virtual_(2)), cos(q_virtual_(2));
        base_vel_.head(2) = rot_base2world * qdot_virtual_.head(2);
        base_vel_(2) = qdot_virtual_(2);

        // get manipulator joint
        for(size_t i=0; i<MANI_DOF; i++)
        {
            const std::string& name = "fr3_joint" + std::to_string(i+1);
            q_mani_(i) = pos_dict.at(name)(0);
            qdot_mani_(i) = vel_dict.at(name)(0);
        }

        // get mobile wheel joint
        q_mobile_(0) = pos_dict.at("front_left_wheel")(0);
        q_mobile_(1) = pos_dict.at("front_right_wheel")(0);
        q_mobile_(2) = pos_dict.at("rear_left_wheel")(0);
        q_mobile_(3) = pos_dict.at("rear_right_wheel")(0);
        qdot_mobile_(0) = vel_dict.at("front_left_wheel")(0);
        qdot_mobile_(1) = vel_dict.at("front_right_wheel")(0);
        qdot_mobile_(2) = vel_dict.at("rear_left_wheel")(0);
        qdot_mobile_(3) = vel_dict.at("rear_right_wheel")(0);

        if(!robot_data_->updateState(q_virtual_, q_mobile_, q_mani_, qdot_virtual_, qdot_mobile_, qdot_mani_))
        {
            RCLCPP_ERROR(node_->get_logger(), "[FR3XLSRobotData] Failed to update robot state.");
        }

        // get ee
        x_ = robot_data_->getPose();
        xdot_ = robot_data_->getVelocity();
    }

    void FR3XLSController::compute()
    {
        if(is_mode_changed_)
        {
            is_mode_changed_ = false;

            control_start_time_ = current_time_;

            base_vel_init_= base_vel_;
            base_vel_desired_.setZero();

            q_virtual_init_ = q_virtual_;
            qdot_virtual_init_ = qdot_virtual_;
            q_virtual_desired_ = q_virtual_init_;
            qdot_virtual_desired_.setZero();

            q_mani_init_ = q_mani_;
            qdot_mani_init_ = qdot_mani_;
            q_mani_desired_ = q_mani_init_;
            qdot_mani_desired_.setZero();
            
            q_mobile_init_ = q_mobile_;
            qdot_mobile_init_ = qdot_mobile_;
            q_mobile_desired_ = q_mobile_init_;
            qdot_mobile_desired_.setZero();

            x_init_ = x_;
            x_desired_ = x_init_;
            x_goal_ = x_init_;

            xdot_init_ = xdot_;
            xdot_desired_.setZero();
        }
        
        if(mode_ == "HOME")
        {
            ManiVec q_mani_target;
            q_mani_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
            
           torque_mani_desired_ = robot_controller_->moveManipulatorJointTorqueCubic(q_mani_target,
                                                                                     ManiVec::Zero(),
                                                                                     q_mani_init_,
                                                                                     qdot_mani_init_,
                                                                                     current_time_,
                                                                                     control_start_time_,
                                                                                     4.0);
            qdot_mobile_desired_.setZero();
        }
        else if(mode_ == "QPIK")
        {
            if(is_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                x_init_ = x_;
                xdot_init_ = xdot_;
                is_goal_pose_changed_ = false;
            }
            VectorXd qdot_mobile_desired, qdot_mani_desired;
            robot_controller_->QPIKCubic(x_goal_,
                                         TaskVec::Zero(),
                                         x_init_,
                                         xdot_init_,
                                         current_time_,
                                         control_start_time_,
                                         4.0,
                                         robot_data_->getEEName(),
                                         qdot_mobile_desired,
                                         qdot_mani_desired);
            qdot_mobile_desired_ = qdot_mobile_desired;
            qdot_mani_desired_ = qdot_mani_desired;
            qdot_mani_desired_ += dt_ * qdot_mani_desired_;
            torque_mani_desired_ = robot_controller_->moveManipulatorJointTorqueStep(q_mani_desired_, qdot_mani_desired_);
        }
        else if(mode_ == "QPID")
        {
            if(is_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                x_init_ = x_;
                xdot_init_ = xdot_;
                is_goal_pose_changed_ = false;
            }
            VectorXd qddot_mobile_desired,torque_mani_desired;
            robot_controller_->QPIDCubic(x_goal_,
                                         TaskVec::Zero(),
                                         x_init_,
                                         xdot_init_,
                                         current_time_,
                                         control_start_time_,
                                         4.0,
                                         robot_data_->getEEName(),
                                         qddot_mobile_desired,
                                         torque_mani_desired);
            torque_mani_desired_ = torque_mani_desired;
            qdot_mobile_desired_ += dt_ * qddot_mobile_desired;
        }
        else
        {
            torque_mani_desired_ = robot_data_->getGravity().segment(robot_data_->getJointIndex().mani_start, MANI_DOF);
            qdot_mobile_desired_.setZero();
        }
    }

    CtrlInputMap FR3XLSController::getCtrlInput() const
    {
        CtrlInputMap ctrl_dict;
        ctrl_dict["front_left_wheel"]  = qdot_mobile_desired_(0);
        ctrl_dict["front_right_wheel"] = qdot_mobile_desired_(1);
        ctrl_dict["rear_left_wheel"]   = qdot_mobile_desired_(2);
        ctrl_dict["rear_right_wheel"]  = qdot_mobile_desired_(3);
        for(size_t i=0; i<7; i++)
        {
            const std::string name = "fr3_joint" + std::to_string(i+1);
            ctrl_dict[name] = torque_mani_desired_(i);
        }

        return ctrl_dict;
    }

    void FR3XLSController::setMode(const std::string& mode)
    {
        is_mode_changed_ = true;
        mode_ = mode;
        RCLCPP_INFO(node_->get_logger(), "\033[34m Mode changed: %s\033[0m", mode.c_str());
    }

    void FR3XLSController::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Key input received: %d", msg->data);
        if(msg->data == 1)      setMode("HOME");
        else if(msg->data == 2) setMode("QPIK");
        else if(msg->data == 3) setMode("QPID");
        else                    setMode("NONE");
    }

    void FR3XLSController::subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

        x_goal_.linear() = quat.toRotationMatrix();
        x_goal_.translation() << msg->pose.position.x,
                                 msg->pose.position.y,
                                 msg->pose.position.z;
    }

    void FR3XLSController::subtargetBaseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Target Base vel received: linear = %.3f, %.3f, angular = %.3f",
                    msg->linear.x, msg->linear.y, msg->angular.z);

        base_vel_desired_(0) = msg->linear.x;
        base_vel_desired_(1) = msg->linear.y;
        base_vel_desired_(2) = msg->angular.z;
    }

    void FR3XLSController::subJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header = msg->header;
        joint_msg.name = msg->name;
        joint_msg.position = msg->position;
        joint_msg.velocity = msg->velocity;
        joint_msg.effort = msg->effort;

        const std::vector<std::string> virtual_joints = {"v_x_joint", "v_y_joint", "v_t_joint"};
        std::vector<double> virtual_pos_vec(q_virtual_.data(), q_virtual_.data() + q_virtual_.size());
        std::vector<double> virtual_vel_vec(qdot_virtual_.data(), qdot_virtual_.data() + qdot_virtual_.size());
        std::vector<double> virtual_eff_vec{0,0,0};

        joint_msg.name.insert(joint_msg.name.end(), virtual_joints.begin(), virtual_joints.end());
        joint_msg.position.insert(joint_msg.position.end(), virtual_pos_vec.begin(), virtual_pos_vec.end());
        joint_msg.velocity.insert(joint_msg.velocity.end(), virtual_vel_vec.begin(), virtual_vel_vec.end());
        joint_msg.effort.insert(joint_msg.effort.end(), virtual_eff_vec.begin(), virtual_eff_vec.end());

        joint_pub_->publish(joint_msg);
    }

    void FR3XLSController::pubEEPoseCallback()
    {
      auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
      ee_pose_msg.header.frame_id = "world";
      ee_pose_msg.header.stamp = node_->now();

      ee_pose_msg.pose.position.x = x_.translation()(0);
      ee_pose_msg.pose.position.y = x_.translation()(1);
      ee_pose_msg.pose.position.z = x_.translation()(2);

      Eigen::Quaterniond q(x_.rotation());
      ee_pose_msg.pose.orientation.x = q.x();
      ee_pose_msg.pose.orientation.y = q.y();
      ee_pose_msg.pose.orientation.z = q.z();
      ee_pose_msg.pose.orientation.w = q.w();
      
      ee_pose_pub_->publish(ee_pose_msg);
    }

    /* register with the global registry */
    REGISTER_MJ_CONTROLLER(FR3XLSController, "FR3XLSController")
}