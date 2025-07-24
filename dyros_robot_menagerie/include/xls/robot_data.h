#pragma once
#include "robot_data/mobile/base.h"

namespace XLS
{

    inline constexpr int TASK_DOF  = 3;
    inline constexpr int WHEEL_DOF = 4;

    typedef Eigen::Matrix<double,TASK_DOF,1>  TaskVec;
    typedef Eigen::Matrix<double,WHEEL_DOF,1> WheelVec;

    class XLSRobotData : public RobotData::Mobile::MobileBase
    {
        public: 
            XLSRobotData();
    };
} // namespace XLS