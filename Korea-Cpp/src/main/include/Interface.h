#pragma once

#include <array>
#include <vector>
#include <deque>

#include "Constants.h"

using Vector2i = std::array<int, 2>;
using Vector2d = std::array<double, 2>;
using Vector3d = std::array<double, 3>;
using Vector5d = std::array<double, 5>;
using Traj = std::vector<std::array<double, 5>>;

struct LaserScan
{
    double angle = 0;
    double range = 0;

    LaserScan(double angle = 0, double range = 0) : angle(angle), range(range) {}
};

namespace Interface
{
    extern bool EMS, SW1, SW2, SW3;
    extern bool SW1_Pressed, SW2_Pressed, SW3_Pressed;
    extern bool LED_G, LED_R;
    
    extern std::vector<LaserScan> laser_scan, laser_scan_storage;
    extern bool laser_scan_flg, laser_scan_flg2;

    extern double GyroYaw;
    extern bool GyroReset;
    extern Vector3d RobotPose;
    extern double Motor[4], Velocity[3];

    extern bool PathPlanning, SignalTest;

    extern bool Autonomous;
}