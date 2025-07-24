#pragma once
#include "robot_data/mobile/base.h"

namespace PCV
{

    inline constexpr int TASK_DOF  = 3;
    inline constexpr int WHEEL_DOF = 8;

    typedef Eigen::Matrix<double,TASK_DOF,1>  TaskVec;
    typedef Eigen::Matrix<double,WHEEL_DOF,1> WheelVec;

    class PCVRobotData : public RobotData::Mobile::MobileBase
    {
        public: 
            PCVRobotData();
    };
} // namespace PCV