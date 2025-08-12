#include "fr3_xls/robot_data.h"

namespace FR3XLS
{
    static RobotData::Mobile::KinematicParam makeMobileParam()
    {
        RobotData::Mobile::KinematicParam p;
        p.type = RobotData::Mobile::DriveType::Mecanum;
        p.wheel_radius = 0.120;
        p.base2wheel_positions = {Vector2d( 0.2225,  0.2045),  // front_left
                                  Vector2d( 0.2225, -0.2045),  // front_right
                                  Vector2d(-0.2225,  0.2045),  // rear_left
                                  Vector2d(-0.2225, -0.2045)}; // rear_right
        p.base2wheel_angles = {0, 0, 0, 0};
        p.roller_angles = {-M_PI/4,  // front_left
                            M_PI/4,  // front_right
                            M_PI/4,  // rear_left
                           -M_PI/4}; // rear_right
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

    FR3XLSRobotData::FR3XLSRobotData()
    : RobotData::MobileManipulator::MobileManipulatorBase(
        makeMobileParam(),
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_xls.urdf",
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_xls.srdf",
        ament_index_cpp::get_package_share_directory("mujoco_ros_sim"),
        makeJointIndex(),
        makeActuatorIndex())
    {
        ee_name_ = "fr3_link8";
    }

    Affine3d FR3XLSRobotData::computePose(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani)
    {
        return MobileManipulatorBase::computePose(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3XLSRobotData::computeJacobian(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani)
    {
        return MobileManipulatorBase::computeJacobian(q_virtual,q_mobile,q_mani,ee_name_);
    }
    MatrixXd FR3XLSRobotData::computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                             const VectorXd& q_mobile,
                                                             const VectorXd& q_mani,
                                                             const VectorXd& qdot_virtual,
                                                             const VectorXd& qdot_mobile,
                                                             const VectorXd& qdot_mani)
    {
        return MobileManipulatorBase::computeJacobianTimeVariation(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    VectorXd FR3XLSRobotData::computeVelocity(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani,
                                                const VectorXd& qdot_virtual,
                                                const VectorXd& qdot_mobile,
                                                const VectorXd& qdot_mani)
    {
        return MobileManipulatorBase::computeVelocity(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    MatrixXd FR3XLSRobotData::computeJacobianActuated(const VectorXd& q_virtual,
                                                        const VectorXd& q_mobile,
                                                        const VectorXd& q_mani)
    {
        return MobileManipulatorBase::computeJacobianActuated(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3XLSRobotData::computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                                     const VectorXd& q_mobile,
                                                                     const VectorXd& q_mani,
                                                                     const VectorXd& qdot_virtual,
                                                                     const VectorXd& qdot_mobile,
                                                                     const VectorXd& qdot_mani)
    {
        return MobileManipulatorBase::computeJacobianTimeVariationActuated(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    RobotData::Manipulator::ManipulabilityResult FR3XLSRobotData::computeManipulability(const VectorXd& q_mani, 
                                                                                          const VectorXd& qdot_mani, 
                                                                                          const bool& with_grad, 
                                                                                          const bool& with_graddot)
    {
        return MobileManipulatorBase::computeManipulability(q_mani,qdot_mani,with_grad,with_graddot,ee_name_);
    }

    Affine3d FR3XLSRobotData::getPose() const
    {
        return MobileManipulatorBase::getPose(ee_name_);
    }

    MatrixXd FR3XLSRobotData::getJacobian()
    {
        return MobileManipulatorBase::getJacobian(ee_name_);
    }

    MatrixXd FR3XLSRobotData::getJacobianTimeVariation()
    {
        return MobileManipulatorBase::getJacobianTimeVariation(ee_name_);
    } 

    VectorXd FR3XLSRobotData::getVelocity()
    {
        return MobileManipulatorBase::getVelocity(ee_name_);
    }   

    RobotData::Manipulator::ManipulabilityResult FR3XLSRobotData::getManipulability(const bool& with_grad, const bool& with_graddot)
    {
        return MobileManipulatorBase::getManipulability(with_grad,with_graddot,ee_name_);
    }
    
    MatrixXd FR3XLSRobotData::getJacobianActuated()
    {
        return MobileManipulatorBase::getJacobianActuated(ee_name_);
    }

    MatrixXd FR3XLSRobotData::getJacobianActuatedTimeVariation()
    {
        return MobileManipulatorBase::getJacobianActuatedTimeVariation(ee_name_);
    }
}