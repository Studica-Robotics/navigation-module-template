#pragma once

#include <array>
#include <atomic>
#include <vector>
#include <mutex>
#include <sstream>
#include <string>
#include <algorithm>
#include <numeric>

#define _USE_MATH_DEFINES
#include <cmath>

#include "studica/Lidar.h"

namespace WSHK
{
namespace _2024
{
class DataContainer
{
public:
    static void set_LiDAR_data(const studica::Lidar::ScanData &scanData, int frontAngle);
    static std::vector<double> get_LiDAR_data(void);
    static std::array<double, 2> polarToCartesian2(double r, double theta_deg);
    static double normAngle(double angle_deg);
    static std::vector<double> rotateVector(std::vector<double> &vec, int index);
private:
    static std::mutex LiDAR_data_mtx;
    static std::vector<double> LiDAR_data;
};
} // namespace _2024
} // namespace WSHK