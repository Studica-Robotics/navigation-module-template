#pragma once

#include <cmath>
#include <algorithm>

namespace Constants
{
    enum{H, L};

    static constexpr int LED_GREEN = 14;
    static constexpr int LED_RED = 15;

    static constexpr int VMX_SWITH[4] = {8, 9, 10, 11};
    static constexpr int TITAN_SWITCH[4][2] = {{0, L}, {2, L}, {2, H}, {0, H}};
    static constexpr bool SWITCH_INV[4] = {false, false, false, false};

    static constexpr int MOTOR[4] = {0, 1, 2, 3};
    static constexpr bool MOTOR_INV[4] = {true, true, true, true};
    static constexpr int RPM = 340;

    static constexpr int LIDAR_ANGLE_OFFSET = -90, LIDAR_ANGLE_MIN = 0, LIDAR_ANGLE_MAX = 360;
    static constexpr int LIDAR_X_OFFSET = -30;
    static constexpr int LIDAR_Y_OFFSET = 0;

    static constexpr int ROBOT_WIDTH = 428;
    static constexpr int ROBOT_HEIGHT = 432;
    static constexpr int ROBOT_CENTER_OFFSET = 0;
    static constexpr int INFLATION_RANGE = 50;

    static constexpr int WHEEL_DIAMETER = 100;
    static constexpr int WHEEL_WIDTH = 392;

    static constexpr int MIN_VELOCITY[2] = {200, 80};
    static constexpr int MAX_VELOCITY[2] = {250, 200};
    static constexpr int ACCELATION_VALUE[2] = {370, 300};
    static constexpr int DECELERATION_DISTANCE = 450;

    static constexpr int LOOK_AHEAD_DISTANCE = 350;
}