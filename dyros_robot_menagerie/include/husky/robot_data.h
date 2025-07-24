#pragma once
#include "robot_data/mobile/base.h"

namespace Husky
{

    inline constexpr int TASK_DOF  = 3;
    inline constexpr int WHEEL_DOF = 2;

    typedef Eigen::Matrix<double,TASK_DOF,1>  TaskVec;
    typedef Eigen::Matrix<double,WHEEL_DOF,1> WheelVec;

    class HuskyRobotData : public RobotData::Mobile::MobileBase
    {
        public: 
            HuskyRobotData();
    };
} // namespace Husky