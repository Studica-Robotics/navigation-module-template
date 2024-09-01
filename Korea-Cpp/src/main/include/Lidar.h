#pragma once

#include <vector>
#include <thread>

#include "Interface.h"

class Lidar
{
public:
    Lidar();
    ~Lidar();

    void Update();

private:
    std::thread thread_object_;
    int lidar_serial_;
    std::vector<LaserScan> laser_scan_tmp_, laser_scan_raw_;
    int lidar_publish_cnt_ = 0;

    int prev_time = 0;

    void RunThread();
    char ReadByte();
};