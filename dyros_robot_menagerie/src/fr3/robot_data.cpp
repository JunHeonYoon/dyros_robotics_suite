#include "fr3/robot_data.h"

namespace FR3
{
    FR3RobotData::FR3RobotData()
    : RobotData::Manipulator::ManipulatorBase(
          ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3.urdf",
          ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3.srdf",
          ament_index_cpp::get_package_share_directory("mujoco_ros_sim")) 
    {
        ee_name_ = "fr3_link8";
    }

    Affine3d FR3RobotData::computePose(const VectorXd& q)
    {
        return ManipulatorBase::computePose(q, ee_name_);
    }

    MatrixXd FR3RobotData::computeJacobian(const VectorXd& q)
    {
        return ManipulatorBase::computeJacobian(q, ee_name_);
    }

    MatrixXd FR3RobotData::computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot)
    {
        return ManipulatorBase::computeJacobianTimeVariation(q, qdot, ee_name_);
    }

    VectorXd FR3RobotData::computeVelocity(const VectorXd& q, const VectorXd& qdot)
    {
        return ManipulatorBase::computeVelocity(q, qdot, ee_name_);
    }

    RobotData::Manipulator::ManipulabilityResult FR3RobotData::computeManipulability(const VectorXd& q, const VectorXd& qdot, const bool& with_grad, const bool& with_graddot)
    {
        return ManipulatorBase::computeManipulability(q, qdot, with_grad, with_graddot, ee_name_);
    }
    
    Affine3d FR3RobotData::getPose() const
    {
        return ManipulatorBase::getPose(ee_name_);
    }

    MatrixXd FR3RobotData::getJacobian()
    {
        return ManipulatorBase::getJacobian(ee_name_);
    }

    MatrixXd FR3RobotData::getJacobianTimeVariation()
    {
        return ManipulatorBase::getJacobianTimeVariation(ee_name_);
    } 

    VectorXd FR3RobotData::getVelocity()
    {
        return ManipulatorBase::getVelocity(ee_name_);
    }

    RobotData::Manipulator::ManipulabilityResult FR3RobotData::getManipulability(const bool& with_grad, const bool& with_graddot)
    {
        return ManipulatorBase::getManipulability(with_grad, with_graddot, ee_name_);
    }
}