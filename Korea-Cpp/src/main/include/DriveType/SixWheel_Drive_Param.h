#pragma once

#include <cmath>
#include <algorithm>

namespace Constants
{
    enum{H, L};

    static constexpr int LED_GREEN = 14;
    static constexpr int LED_RED = 15;

    static constexpr int VMX_SWITH[4] = {8, 9, 10, 11};
    static constexpr int TITAN_SWITCH[4][2] = {{0, L}, {1, L}, {1, H}, {0, H}};
    static constexpr bool SWITCH_INV[4] = {false, false, false, false};

    static constexpr int MOTOR[4] = {2, 0, 3, 1};
    static constexpr bool MOTOR_INV[4] = {false, false, false, false};
    static constexpr int RPM = 100;

    static constexpr int LIDAR_ANGLE_OFFSET = 90, LIDAR_ANGLE_MIN = 0, LIDAR_ANGLE_MAX = 360;
    static constexpr int LIDAR_X_OFFSET = 8 + 48 * 4;
    static constexpr int LIDAR_Y_OFFSET = 0;

    static constexpr int ROBOT_WIDTH = 436 - 8; // 431 446
    static constexpr int ROBOT_HEIGHT = 432;
    static constexpr int ROBOT_CENTER_OFFSET = 48 * 4; // 120 + 72 + 36 48 * 4
    static constexpr int INFLATION_RANGE = 50;

    static constexpr int WHEEL_DIAMETER = 100;
    static constexpr int WHEEL_WIDTH = 380;

    static constexpr int MIN_VELOCITY[2] = {50, 40};
    static constexpr int MAX_VELOCITY[2] = {200, 100};
    static constexpr int ACCELATION_VALUE[2] = {250, 200};
    static constexpr int DECELERATION_DISTANCE = 400;

    static constexpr int LOOK_AHEAD_DISTANCE = 450;
}