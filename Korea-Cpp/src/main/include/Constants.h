#pragma once

#include <cmath>

#define X_Drive 0
#define Mecanum_Drive 1
#define Diff_Drive 2
#define Tire_Drive 3
#define SixWheel_Drive 4
#define Omni3_Drive 5

#define DRIVE_TYPE Tire_Drive

#if (DRIVE_TYPE == X_Drive)
    #include "DriveType/X_Drive_Param.h"
#elif (DRIVE_TYPE == Mecanum_Drive)
    #include "DriveType/Mecanum_Drive_Param.h" 
#elif (DRIVE_TYPE == Diff_Drive)
    #include "DriveType/Diff_Drive_Param.h"
#elif (DRIVE_TYPE == Tire_Drive)
    #include "DriveType/Tire_Drive_Param.h"
#elif (DRIVE_TYPE == SixWheel_Drive)
    #include "DriveType/SixWheel_Drive_Param.h"
#elif (DRIVE_TYPE == Omni3_Drive)
    #include "DriveType/Omni3_Drive_Param.h"
#endif

namespace Constants
{
    static constexpr bool DEBUG = true;

    static constexpr int MapBorder[2] = {4000, 1500};
    static constexpr int Start[2] = {4250 + ROBOT_CENTER_OFFSET, 750};

    static constexpr int Goal[2] = {4700 + ROBOT_CENTER_OFFSET + (DRIVE_TYPE == SixWheel_Drive ? 100 : 0), 0};

    static constexpr int LIDAR_DATA_CUT_SIZE = 400;
    
    static constexpr int LIDAR_MIN_RANGE = 100;
    static constexpr int LIDAR_MAX_RANGE = 6000;

    static constexpr int MAP_SIZE = 320;

    static constexpr int MM_PER_PIXEL = 18;
    static constexpr int OCCUPIED_GAIN = -30, FREE_GAIN = 30;
    static constexpr int GOAL_TORLERANCE = 100;
    
    static constexpr int MapStartPoint[2] =
    {
        int(std::round((MAP_SIZE - MapBorder[0] / double(Constants::MM_PER_PIXEL)) / 2.0)),
        int(std::round((MAP_SIZE - MapBorder[1] / double(Constants::MM_PER_PIXEL)) / 2.0))
    };
    static constexpr int ROBOT_X_OFFSET = MapStartPoint[0] + Start[0] / double(Constants::MM_PER_PIXEL);
    static constexpr int ROBOT_Y_OFFSET = MapStartPoint[1] + Start[1] / double(Constants::MM_PER_PIXEL);

    static constexpr double ROBOT_RADIUS = std::sqrt(ROBOT_WIDTH * ROBOT_WIDTH + ROBOT_HEIGHT * ROBOT_HEIGHT) / 2.0 / double(MM_PER_PIXEL);
    
    static constexpr double INFLATION_RADIUS = ROBOT_RADIUS + INFLATION_RANGE / double(MM_PER_PIXEL);

    static constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
}