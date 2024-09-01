#pragma once

#include <cmath>
#include <algorithm>

namespace Constants
{
    enum{H, L};

    static constexpr int LED_GREEN = 14;
    static constexpr int LED_RED = 15;

    static constexpr int VMX_SWITH[4] = {8, 9, 10, 11};
    static constexpr int TITAN_SWITCH[4][2] = {{1, L}, {3, H}, {3, L}, {0, H}};
    static constexpr bool SWITCH_INV[4] = {true, false, false, false};
    
    static constexpr int MOTOR[4] = {2, 3, 0, 1};
    static constexpr bool MOTOR_INV[4] = {false, false, false, false};
    static constexpr int RPM = 100;

    static constexpr int LIDAR_ANGLE_OFFSET = 90, LIDAR_ANGLE_MIN = 0, LIDAR_ANGLE_MAX = 360;
    static constexpr int LIDAR_X_OFFSET = 13;
    static constexpr int LIDAR_Y_OFFSET = 0;

    static constexpr int ROBOT_WIDTH = 312;
    static constexpr int ROBOT_HEIGHT = 346;
    static constexpr int ROBOT_CENTER_OFFSET = 0;
    static constexpr int INFLATION_RANGE = 50;

    static constexpr int WHEEL_DIAMETER = 100;
    static constexpr int WHEEL_WIDTH = 288;

    static constexpr int MIN_VELOCITY[2] = {30, 20};
    static constexpr int MAX_VELOCITY[2] = {200, 60};
    static constexpr int ACCELATION_VALUE[2] = {250, 200};
    static constexpr int DECELERATION_DISTANCE = 400;

    static constexpr int LOOK_AHEAD_DISTANCE = 350;
}