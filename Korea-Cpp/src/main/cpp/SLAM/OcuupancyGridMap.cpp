#include <opencv2/imgproc/imgproc.hpp>

#include "SLAM/OcuupancyGridMap.h"
#include "Util.h"
#include "Constants.h"

OcuupancyGridMap::OcuupancyGridMap(unsigned char **map) : map_(map){ }
void OcuupancyGridMap::ResetMap()
{
    for (int i = 0; i < Constants::MAP_SIZE; i++)
        for (int j = 0; j < Constants::MAP_SIZE; j++)
            map_[i][j] = 128;

    if(DRIVE_TYPE == Tire_Drive)
    {
        int grid_map_border[2] = 
        {
            int(std::round(Constants::MapBorder[0] / double(Constants::MM_PER_PIXEL))),
            int(std::round(Constants::MapBorder[1] / double(Constants::MM_PER_PIXEL))),
        };
        for(int i = 0; i < grid_map_border[1]; i++)
        {
            if(std::round((Constants::Start[1] - 300) / double(Constants::MM_PER_PIXEL)) <= i && 
                std::round((Constants::Start[1] + 300) / double(Constants::MM_PER_PIXEL)) >= i) continue;
            map_[Constants::MapStartPoint[0] + grid_map_border[0]][Constants::MapStartPoint[1] + i] = 0;
        }
    }
}
void OcuupancyGridMap::Update()
{
    Vector2i grid_robot_pose = ConvertToGridCoordinate(Interface::RobotPose);

    double r, th;
    for (int i = 0; i < 450; i++)
    {
        if(Interface::laser_scan[i].range)
        {
            r = Interface::laser_scan[i].range;
            th = NormalizeRadian(Interface::laser_scan[i].angle + Interface::RobotPose[2]);

            Vector3d laser_point;
            laser_point[0] = r * std::cos(th) + Interface::RobotPose[0];
            laser_point[1] = r * std::sin(th) + Interface::RobotPose[1];
            laser_point[2] = th;
            
            if(laser_point[0] <= 100 + Constants::ROBOT_CENTER_OFFSET) continue;
            if(laser_point[0] >= Constants::MapBorder[0] + Constants::ROBOT_CENTER_OFFSET + Constants::LIDAR_DATA_CUT_SIZE) continue;
            if(laser_point[1] <= -Constants::MapBorder[1] / 2.0 - Constants::LIDAR_DATA_CUT_SIZE) continue;
            if(laser_point[1] >= Constants::MapBorder[1] / 2.0 + Constants::LIDAR_DATA_CUT_SIZE) continue;

            Vector2i grid_pose = ConvertToGridCoordinate(laser_point);
            if (IsSafe(grid_pose, Constants::MAP_SIZE))
                Bresenham(grid_robot_pose, grid_pose);
        }
    }
}
void OcuupancyGridMap::Bresenham(const Vector2i pt0, const Vector2i pt1)
{
    if (!IsSafe(pt0, Constants::MAP_SIZE) || !IsSafe(pt1, Constants::MAP_SIZE))
        return;

    int direction_x = pt1[0] - pt0[0];
    int direction_y = pt1[1] - pt0[1];
    int delta_x = std::abs(direction_x);
    int delta_y = std::abs(direction_y);
    int max_xy = delta_x > delta_y ? delta_x : delta_y;
    int decision_param_x = -max_xy / 2;
    int decision_param_y = decision_param_x;

    int x = pt0[0], y = pt0[1];
    if (IsSafe(Vector2i{x, y}, Constants::MAP_SIZE) && map_[x][y] > 1)
    {
        if (map_[x][y] + Constants::FREE_GAIN > 255)
            map_[x][y] = 255;
        else
            map_[x][y] += Constants::FREE_GAIN;
    }

    for (int i = 0; i < max_xy; ++i)
    {
        // process x
        decision_param_x += delta_x;
        if (decision_param_x > 0)
        {
            direction_x > 0 ? x++ : x--;
            decision_param_x -= max_xy;
        }
        // process y
        decision_param_y += delta_y;
        if (decision_param_y > 0)
        {
            direction_y > 0 ? y++ : y--;
            decision_param_y -= max_xy;
        }

        if (IsSafe(Vector2i{x, y}, Constants::MAP_SIZE) && map_[x][y] > 1)
        {
            if (i == max_xy - 1)
            {
                if (map_[x][y] + Constants::OCCUPIED_GAIN < 1)
                    map_[x][y] = 1;
                else
                    map_[x][y] += Constants::OCCUPIED_GAIN;
            }
            else
            {
                if (map_[x][y] + Constants::FREE_GAIN > 255)
                    map_[x][y] = 255;
                else
                    map_[x][y] += Constants::FREE_GAIN;
            }
        }
    }
}
void OcuupancyGridMap::Show(cv::Mat &mat)
{
    // Map
    for (int i = 0; i < Constants::MAP_SIZE; i++)
    {
        for (int j = 0; j < Constants::MAP_SIZE; j++)
        {
            mat.data[i * mat.cols * 3 + j * 3 + 0] = map_[i][j];
            mat.data[i * mat.cols * 3 + j * 3 + 1] = map_[i][j];
            mat.data[i * mat.cols * 3 + j * 3 + 2] = map_[i][j];
        }
    }
}