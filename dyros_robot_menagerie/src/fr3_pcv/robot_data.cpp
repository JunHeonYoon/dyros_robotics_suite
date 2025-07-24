#include "fr3_pcv/robot_data.h"

namespace FR3PCV
{
    static RobotData::Mobile::KinematicParam makeMobileParam()
    {
        RobotData::Mobile::KinematicParam p;
        p.type = RobotData::Mobile::DriveType::Caster;
        p.wheel_radius = 0.055;
        p.wheel_offset = 0.020;
        p.base2wheel_positions = {Vector2d( 0.215,  0.125),  // front_left
                                  Vector2d( 0.215, -0.125),  // front_right
                                  Vector2d(-0.215,  0.125),  // rear_left
                                  Vector2d(-0.215, -0.125)}; // rear_right
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

    FR3PCVRobotData::FR3PCVRobotData()
    : RobotData::MobileManipulator::MobileManipulatorBase(
        makeMobileParam(),
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_pcv.urdf",
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_pcv.srdf",
        ament_index_cpp::get_package_share_directory("mujoco_ros_sim"),
        makeJointIndex(),
        makeActuatorIndex(),
        true)
    {
        ee_name_ = "fr3_link8";
    }

    Affine3d FR3PCVRobotData::computePose(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani)
    {
        return MobileManipulatorBase::computePose(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3PCVRobotData::computeJacobian(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani)
    {
        return MobileManipulatorBase::computeJacobian(q_virtual,q_mobile,q_mani,ee_name_);
    }
    MatrixXd FR3PCVRobotData::computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                             const VectorXd& q_mobile,
                                                             const VectorXd& q_mani,
                                                             const VectorXd& qdot_virtual,
                                                             const VectorXd& qdot_mobile,
                                                             const VectorXd& qdot_mani)
    {
        return MobileManipulatorBase::computeJacobianTimeVariation(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    VectorXd FR3PCVRobotData::computeVelocity(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani,
                                                const VectorXd& qdot_virtual,
                                                const VectorXd& qdot_mobile,
                                                const VectorXd& qdot_mani)
    {
        return MobileManipulatorBase::computeVelocity(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    MatrixXd FR3PCVRobotData::computeJacobianActuated(const VectorXd& q_virtual,
                                                        const VectorXd& q_mobile,
                                                        const VectorXd& q_mani)
    {
        return MobileManipulatorBase::computeJacobianActuated(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3PCVRobotData::computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                                     const VectorXd& q_mobile,
                                                                     const VectorXd& q_mani,
                                                                     const VectorXd& qdot_virtual,
                                                                     const VectorXd& qdot_mobile,
                                                                     const VectorXd& qdot_mani)
    {
        return MobileManipulatorBase::computeJacobianTimeVariationActuated(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    RobotData::Manipulator::ManipulabilityResult FR3PCVRobotData::computeManipulability(const VectorXd& q_mani, 
                                                                                          const VectorXd& qdot_mani, 
                                                                                          const bool& with_grad, 
                                                                                          const bool& with_graddot)
    {
        return MobileManipulatorBase::computeManipulability(q_mani,qdot_mani,with_grad,with_graddot,ee_name_);
    }

    Affine3d FR3PCVRobotData::getPose() const
    {
        return MobileManipulatorBase::getPose(ee_name_);
    }

    MatrixXd FR3PCVRobotData::getJacobian()
    {
        return MobileManipulatorBase::getJacobian(ee_name_);
    }

    MatrixXd FR3PCVRobotData::getJacobianTimeVariation()
    {
        return MobileManipulatorBase::getJacobianTimeVariation(ee_name_);
    } 

    VectorXd FR3PCVRobotData::getVelocity()
    {
        return MobileManipulatorBase::getVelocity(ee_name_);
    }   

    RobotData::Manipulator::ManipulabilityResult FR3PCVRobotData::getManipulability(const bool& with_grad, const bool& with_graddot)
    {
        return MobileManipulatorBase::getManipulability(with_grad,with_graddot,ee_name_);
    }
    
    MatrixXd FR3PCVRobotData::getJacobianActuated()
    {
        return MobileManipulatorBase::getJacobianActuated(ee_name_);
    }

    MatrixXd FR3PCVRobotData::getJacobianActuatedTimeVariation()
    {
        return MobileManipulatorBase::getJacobianActuatedTimeVariation(ee_name_);
    }
}