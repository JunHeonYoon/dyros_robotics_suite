#include "pcv/robot_data.h"

namespace PCV
{
    static RobotData::Mobile::KinematicParam makeParam()
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

    PCVRobotData::PCVRobotData()
    : RobotData::Mobile::MobileBase(makeParam())
    {
        
    }
}