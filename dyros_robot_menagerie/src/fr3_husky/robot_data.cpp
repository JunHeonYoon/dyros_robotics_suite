#include "fr3_husky/robot_data.h"

namespace FR3Husky
{
    static RobotData::Mobile::KinematicParam makeMobileParam()
    {
        RobotData::Mobile::KinematicParam p;
        p.type         = RobotData::Mobile::DriveType::Differential;
        p.wheel_radius = 0.1651;
        p.base_width   = 0.2854 * 2 * 1.875;
        return p;
    }

    static RobotData::MobileManipulator::JointIndex makeJointIndex()
    {
        RobotData::MobileManipulator::JointIndex j;
        j.virtual_start = 0;
        j.mani_start = VIRTUAL_DOF;
        j.mobi_start = VIRTUAL_DOF + MANI_DOF;
        return j;
    }

    static RobotData::MobileManipulator::ActuatorIndex makeActuatorIndex()
    {
        RobotData::MobileManipulator::ActuatorIndex a;
        a.mani_start = 0;
        a.mobi_start = MANI_DOF;
        return a;
    }

    FR3HuskyRobotData::FR3HuskyRobotData()
    : RobotData::MobileManipulator::MobileManipulatorBase(
        makeMobileParam(),
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_husky.urdf",
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_husky.srdf",
        ament_index_cpp::get_package_share_directory("mujoco_ros_sim"),
        makeJointIndex(),
        makeActuatorIndex())
    {
        ee_name_ = "fr3_link8";
    }

    Affine3d FR3HuskyRobotData::computePose(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani)
    {
        return MobileManipulatorBase::computePose(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3HuskyRobotData::computeJacobian(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani)
    {
        return MobileManipulatorBase::computeJacobian(q_virtual,q_mobile,q_mani,ee_name_);
    }
    MatrixXd FR3HuskyRobotData::computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                             const VectorXd& q_mobile,
                                                             const VectorXd& q_mani,
                                                             const VectorXd& qdot_virtual,
                                                             const VectorXd& qdot_mobile,
                                                             const VectorXd& qdot_mani)
    {
        return MobileManipulatorBase::computeJacobianTimeVariation(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    VectorXd FR3HuskyRobotData::computeVelocity(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani,
                                                const VectorXd& qdot_virtual,
                                                const VectorXd& qdot_mobile,
                                                const VectorXd& qdot_mani)
    {
        return MobileManipulatorBase::computeVelocity(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    MatrixXd FR3HuskyRobotData::computeJacobianActuated(const VectorXd& q_virtual,
                                                        const VectorXd& q_mobile,
                                                        const VectorXd& q_mani)
    {
        return MobileManipulatorBase::computeJacobianActuated(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3HuskyRobotData::computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                                     const VectorXd& q_mobile,
                                                                     const VectorXd& q_mani,
                                                                     const VectorXd& qdot_virtual,
                                                                     const VectorXd& qdot_mobile,
                                                                     const VectorXd& qdot_mani)
    {
        return MobileManipulatorBase::computeJacobianTimeVariationActuated(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    RobotData::Manipulator::ManipulabilityResult FR3HuskyRobotData::computeManipulability(const VectorXd& q_mani, 
                                                                                          const VectorXd& qdot_mani, 
                                                                                          const bool& with_grad, 
                                                                                          const bool& with_graddot)
    {
        return MobileManipulatorBase::computeManipulability(q_mani,qdot_mani,with_grad,with_graddot,ee_name_);
    }

    Affine3d FR3HuskyRobotData::getPose() const
    {
        return MobileManipulatorBase::getPose(ee_name_);
    }

    MatrixXd FR3HuskyRobotData::getJacobian()
    {
        return MobileManipulatorBase::getJacobian(ee_name_);
    }

    MatrixXd FR3HuskyRobotData::getJacobianTimeVariation()
    {
        return MobileManipulatorBase::getJacobianTimeVariation(ee_name_);
    } 

    VectorXd FR3HuskyRobotData::getVelocity()
    {
        return MobileManipulatorBase::getVelocity(ee_name_);
    }   

    RobotData::Manipulator::ManipulabilityResult FR3HuskyRobotData::getManipulability(const bool& with_grad, const bool& with_graddot)
    {
        return MobileManipulatorBase::getManipulability(with_grad,with_graddot,ee_name_);
    }
    
    MatrixXd FR3HuskyRobotData::getJacobianActuated()
    {
        return MobileManipulatorBase::getJacobianActuated(ee_name_);
    }

    MatrixXd FR3HuskyRobotData::getJacobianActuatedTimeVariation()
    {
        return MobileManipulatorBase::getJacobianActuatedTimeVariation(ee_name_);
    }
}