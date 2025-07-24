#pragma once
#include "robot_data/mobile_manipulator/base.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace FR3PCV
{
    /*
    FR3 PCV URDF Joint Information
    Total nq = 18
    Total nv = 18

     id | name                | nq | nv | idx_q | idx_v
    ----+---------------------+----+----+-------+------
      1 |           v_x_joint |  1 |  1 |     0 |    0
      2 |           v_y_joint |  1 |  1 |     1 |    1
      3 |           v_t_joint |  1 |  1 |     2 |    2
      4 |          fr3_joint1 |  1 |  1 |     3 |    3
      5 |          fr3_joint2 |  1 |  1 |     4 |    4
      6 |          fr3_joint3 |  1 |  1 |     5 |    5
      7 |          fr3_joint4 |  1 |  1 |     6 |    6
      8 |          fr3_joint5 |  1 |  1 |     7 |    7
      9 |          fr3_joint6 |  1 |  1 |     8 |    8
     10 |          fr3_joint7 |  1 |  1 |     9 |    9
     11 |    front_left_steer |  1 |  1 |    10 |   10
     12 |   front_left_rotate |  1 |  1 |    11 |   11
     13 |   front_right_steer |  1 |  1 |    12 |   12
     14 |  front_right_rotate |  1 |  1 |    13 |   13
     15 |     rear_left_steer |  1 |  1 |    14 |   14
     16 |    rear_left_rotate |  1 |  1 |    15 |   15
     17 |    rear_right_steer |  1 |  1 |    16 |   16
     18 |   rear_right_rotate |  1 |  1 |    17 |   17
    */

    inline constexpr int TASK_DOF     = 6;
    inline constexpr int VIRTUAL_DOF  = 3;
    inline constexpr int MANI_DOF     = 7;
    inline constexpr int MOBI_DOF     = 8;
    inline constexpr int ACTUATOR_DOF = MANI_DOF + MOBI_DOF;
    inline constexpr int JOINT_DOF    = ACTUATOR_DOF + VIRTUAL_DOF;

    typedef Eigen::Matrix<double,TASK_DOF,1>     TaskVec;
    typedef Eigen::Matrix<double,VIRTUAL_DOF,1>  VirtualVec;
    typedef Eigen::Matrix<double,MANI_DOF,1>     ManiVec;
    typedef Eigen::Matrix<double,MOBI_DOF,1>     MobiVec;
    typedef Eigen::Matrix<double,ACTUATOR_DOF,1> AactuatorVec;
    typedef Eigen::Matrix<double,JOINT_DOF,1>    JointVec;

    class FR3PCVRobotData : public RobotData::MobileManipulator::MobileManipulatorBase
    {
        public:
            FR3PCVRobotData();

            Affine3d computePose(const VectorXd& q_virtual,
                                 const VectorXd& q_mobile,
                                 const VectorXd& q_mani);
            MatrixXd computeJacobian(const VectorXd& q_virtual,
                                     const VectorXd& q_mobile,
                                     const VectorXd& q_mani);
            MatrixXd computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                  const VectorXd& q_mobile,
                                                  const VectorXd& q_mani,
                                                  const VectorXd& qdot_virtual,
                                                  const VectorXd& qdot_mobile,
                                                  const VectorXd& qdot_mani);
            VectorXd computeVelocity(const VectorXd& q_virtual,
                                     const VectorXd& q_mobile,
                                     const VectorXd& q_mani,
                                     const VectorXd& qdot_virtual,
                                     const VectorXd& qdot_mobile,
                                     const VectorXd& qdot_mani);

            MatrixXd computeJacobianActuated(const VectorXd& q_virtual,
                                             const VectorXd& q_mobile,
                                             const VectorXd& q_mani);
            MatrixXd computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                          const VectorXd& q_mobile,
                                                          const VectorXd& q_mani,
                                                          const VectorXd& qdot_virtual,
                                                          const VectorXd& qdot_mobile,
                                                          const VectorXd& qdot_mani);

            RobotData::Manipulator::ManipulabilityResult computeManipulability(const VectorXd& q_mani, 
                                                                               const VectorXd& qdot_mani, 
                                                                               const bool& with_grad, 
                                                                               const bool& with_graddot);
            Affine3d getPose() const;
            MatrixXd getJacobian();
            MatrixXd getJacobianTimeVariation(); 
            VectorXd getVelocity();
            RobotData::Manipulator::ManipulabilityResult getManipulability(const bool& with_grad, const bool& with_graddot);
            MatrixXd getJacobianActuated();
            MatrixXd getJacobianActuatedTimeVariation();
            std::string getEEName(){return ee_name_;}

        private:
            std::string ee_name_;
    };
} // namespace FR3PCV