#include "functions/DataContainer.h"

using namespace WSHK::_2024;

std::vector<double> DataContainer::LiDAR_data(360);

std::mutex DataContainer::LiDAR_data_mtx;

std::vector<double> DataContainer::get_LiDAR_data(void)
{
    std::lock_guard<std::mutex> lock(LiDAR_data_mtx);
    return LiDAR_data;
}

void DataContainer::set_LiDAR_data(const studica::Lidar::ScanData &scanData, int frontAngle)
{
    std::lock_guard<std::mutex> lock(LiDAR_data_mtx);
    std::vector<double> LiDAR_data_temp(360);
    std::copy(scanData.distance, scanData.distance + 360, LiDAR_data_temp.begin());
    frontAngle = frontAngle % 360; // Get the remainder of angle divided by 360
    if (frontAngle < 0)
    {
        frontAngle += 360; // Adjust if the angle is negative
    }
    LiDAR_data = rotateVector(LiDAR_data_temp, frontAngle);
}

// Function to convert polar coordinates to Cartesian coordinates
std::array<double, 2> DataContainer::polarToCartesian2(double r, double theta_deg)
{
    // Adjust the angle by front angle degrees
    // double adjusted_theta_deg = 360 - (theta_deg - 90);
    double adjusted_theta_deg = (theta_deg - 90);
    adjusted_theta_deg = normAngle(adjusted_theta_deg);
    double theta_rad = adjusted_theta_deg * M_PI / 180.0; // Convert angle from degrees to radians
    double x = r * std::cos(theta_rad);
    double y = r * std::sin(theta_rad);
    return {x, y};
}

// Function to normalize the negative angle or angle greater than 360 degree
double DataContainer::normAngle(double angle_deg)
{
    // If the angle is negative, add 360 until it's positive
    while (angle_deg < 0)
    {
        angle_deg += 360;
    }
    // If the angle is greater than or equal to 360, subtract 360 until it's less than 360
    while (angle_deg >= 360)
    {
        angle_deg -= 360;
    }
    return angle_deg;
}

std::vector<double> DataContainer::rotateVector(std::vector<double> &vec, int index)
{
    std::vector<double> result(vec.begin(), vec.end());
    std::rotate(result.begin(), result.begin() + index, result.end());
    //  Logger::log("rotate: " + vec2str(result));
    return result;
}
