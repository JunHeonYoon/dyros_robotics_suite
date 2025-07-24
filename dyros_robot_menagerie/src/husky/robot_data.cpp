#include "husky/robot_data.h"

namespace Husky
{
    static RobotData::Mobile::KinematicParam makeParam()
    {
        RobotData::Mobile::KinematicParam p;
        p.type         = RobotData::Mobile::DriveType::Differential;
        p.wheel_radius = 0.1651;
        p.base_width   = 0.2854 * 2 * 1.875;
        return p;
    }

    HuskyRobotData::HuskyRobotData()
    : RobotData::Mobile::MobileBase(makeParam())
    {

    }
}