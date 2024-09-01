#include <networktables/NetworkTableInstance.h>
#include <frc/DriverStation.h>

#include "Shuffleboard.h"
#include "Util.h"
#include "Interface.h"

Shuffleboard::Shuffleboard()
{
    table_ = nt::NetworkTableInstance::GetDefault().GetTable("Table");
    
    delay(1000, 0);
    for (int i = 0; i < table_cnt_; i++)
        table_arr_[i] = table_->GetEntry(table_str_[i]);
    for (int i = 0; i < table_cnt_; i++)
    {
        if (i < bool_table_cnt_)
            table_arr_[i].SetBoolean(false);
        else if (i < bool_table_cnt_ + double_table_cnt_)
            table_arr_[i].SetDouble(0);
        else
            table_arr_[i].SetString("");
    }
}
void Shuffleboard::Update()
{
    if(!Constants::DEBUG) return;
    
    table_arr_[kEMS].SetBoolean(Interface::EMS);
    table_arr_[kSW1].SetBoolean(Interface::SW1);
    table_arr_[kSW2].SetBoolean(Interface::SW2);
    table_arr_[kSW3].SetBoolean(Interface::SW3);
    table_arr_[kPathPlanning].SetBoolean(Interface::PathPlanning);

    table_arr_[kVoltage].SetDouble(std::round(frc::DriverStation::GetInstance().GetBatteryVoltage() * 100) / 100);
    table_arr_[kRobotPoseX].SetDouble(Interface::RobotPose[0]);
    table_arr_[kRobotPoseY].SetDouble(Interface::RobotPose[1]);
    table_arr_[kRobotPoseW].SetDouble(ToDegree(Interface::RobotPose[2]));
    table_arr_[kGyro].SetDouble(ToDegree(Interface::GyroYaw));

    table_arr_[kVelocityX].SetDouble(Interface::Velocity[0]);
    table_arr_[kVelocityY].SetDouble(Interface::Velocity[1]);
    table_arr_[kVelocityW].SetDouble(Interface::Velocity[2]);

    Interface::SignalTest = table_arr_[kSignalTest].GetBoolean(false);
    if(Interface::SignalTest)
    {
        Interface::LED_G = table_arr_[kLED_Green].GetBoolean(false);
        Interface::LED_R = table_arr_[kLED_Red].GetBoolean(false);
        for(int i = 0; i < 4; i++)
        {
            if(!table_arr_[kMotorEnable0 + i].GetBoolean(false)) table_arr_[kMotor0 + i].SetDouble(0);
            Interface::Motor[i] = table_arr_[kMotor0 + i].GetDouble(false);
        }
        LaserScanMeasure(0);
        LaserScanMeasure(-90);
        LaserScanMeasure(90);
    }
    else
    {
        table_arr_[kLED_Green].SetBoolean(Interface::LED_G);
        table_arr_[kLED_Red].SetBoolean(Interface::LED_R);
        for(int i = 0; i < 4; i++)
            table_arr_[kMotor0 + i].SetDouble(Interface::Motor[i]);
    }
}
void Shuffleboard::LaserScanMeasure(int target)
{
    int cnt = 0;
    double sum = 0;
    for(int i = -7; i < 8; i++)
    {
        int deg = i + target;
        if(deg < 0) deg += 450;

        if(Interface::laser_scan[deg].range)
        {
            sum += Interface::laser_scan[deg].range;
            cnt++;
        }
    }
    if(cnt)
    {
        if(target == 0) table_arr_[kLidarForward].SetDouble(std::round(sum / cnt - Constants::ROBOT_HEIGHT / 2.0));
        else if(target == -90) table_arr_[kLidarLeft].SetDouble(std::round(sum / cnt - Constants::ROBOT_WIDTH / 2.0));
        else if(target == 90) table_arr_[kLidarRight].SetDouble(std::round(sum / cnt - Constants::ROBOT_WIDTH / 2.0));
    }
}