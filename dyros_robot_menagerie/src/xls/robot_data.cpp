#include "xls/robot_data.h"

namespace XLS
{
    static RobotData::Mobile::KinematicParam makeParam()
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

    XLSRobotData::XLSRobotData()
    : RobotData::Mobile::MobileBase(makeParam())
    {
        
    }
}