#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cmath>

#include <thread>
#include <chrono>

#include "Lidar.h"
#include "Constants.h"
#include "Util.h"

Lidar::Lidar()
{
    do
    {
        lidar_serial_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
        if (lidar_serial_ == -1)
            cout << "Lidar Serial Open Failed!" << endl;
        delay(500, 0);
    } while (lidar_serial_ < 0);

    struct termios options;
    tcgetattr(lidar_serial_, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;
    tcflush(lidar_serial_, TCIFLUSH);
    tcsetattr(lidar_serial_, TCSANOW, &options);

    thread_object_ = std::thread(&Lidar::RunThread, this);
}
Lidar::~Lidar()
{
    thread_object_.join();
    close(lidar_serial_);
}
void Lidar::RunThread()
{
    while (true)
    {
        Update();
    }
}
void Lidar::Update()
{
    int CT, LSN, FSA, LSA, CS, check_sum;
    double start_angle, step_angle, end_angle;
    double ang, dis;

    while(ReadByte() & ~0xAA); // PH LSB
    while(ReadByte() & ~0x55); // PH MSB

    check_sum = 0x55AA;

    CT = ReadByte();

    LSN = ReadByte();
    if (LSN == 0)
        return;

    FSA = ReadByte() | (ReadByte() << 8);
    start_angle = (FSA >> 1) / 64;
    check_sum ^= FSA;

    LSA = ReadByte() | (ReadByte() << 8);
    end_angle = (LSA >> 1) / 64;

    if (start_angle == end_angle)
        return;

    CS = ReadByte() | (ReadByte() << 8);

    if (end_angle < start_angle)
        step_angle = (end_angle + 360 - start_angle) / (LSN - 1);
    else
        step_angle = (end_angle - start_angle) / (LSN - 1);

    laser_scan_tmp_.clear();
    for (int i = 0; i < LSN; i++)
    {
        dis = ReadByte() | (ReadByte() << 8);
        check_sum ^= int(dis);
        dis = dis / 4.0;
        if (dis > Constants::LIDAR_MIN_RANGE && dis < Constants::LIDAR_MAX_RANGE)
        {
            ang = start_angle + ToDegree(std::atan(21.8 * ((155.3 - dis) / (155.3 * dis)))) + Constants::LIDAR_ANGLE_OFFSET;
            if (ang < 0)
                ang += 360;
            else if (ang >= 360)
                ang -= 360;
            if(ang >= Constants::LIDAR_ANGLE_MIN || ang <= Constants::LIDAR_ANGLE_MAX)
                laser_scan_tmp_.push_back({ToRadian(ang), dis});
        }
        start_angle += step_angle;
        if (start_angle >= 360)
            start_angle -= 360;
    }

    check_sum ^= CT | (LSN << 8);
    check_sum ^= LSA;

    if (check_sum != CS || laser_scan_tmp_.empty()) return;

    laser_scan_raw_.reserve(laser_scan_raw_.size() + laser_scan_tmp_.size());
    laser_scan_raw_.insert(laser_scan_raw_.end(), laser_scan_tmp_.begin(), laser_scan_tmp_.end());

    lidar_publish_cnt_++;
    if(lidar_publish_cnt_ >= 5)
    {
        for(int i = 0; i < 450; i++) Interface::laser_scan[i].angle = Interface::laser_scan[i].range = 0;
        for(auto scan : laser_scan_raw_)
        {
            int deg = std::round(ToDegree(scan.angle) * 1.25);
            if(deg >= 450) deg = 449;

            double x = scan.range * std::cos(scan.angle) + Constants::LIDAR_X_OFFSET;
            double y = scan.range * std::sin(scan.angle) + Constants::LIDAR_Y_OFFSET;
            
            Interface::laser_scan[deg].angle = std::atan2(y, x);
            Interface::laser_scan[deg].range = std::sqrt(x * x + y * y);

            Interface::laser_scan_storage[deg].angle = Interface::laser_scan[deg].angle;
            Interface::laser_scan_storage[deg].range = Interface::laser_scan[deg].range;
        }
        laser_scan_raw_.clear();
        lidar_publish_cnt_ = 0;
        Interface::laser_scan_flg = true;
        Interface::laser_scan_flg2 = true;
    }
}
char Lidar::ReadByte()
{
    char buf;
    read(lidar_serial_, &buf, 1);
    return buf;
}