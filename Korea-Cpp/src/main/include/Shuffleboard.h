#pragma once

#include <networktables/NetworkTable.h>

class Shuffleboard
{
public:
    Shuffleboard();
    void Update();

private:
    static constexpr int bool_table_cnt_ = 12;
    static constexpr int double_table_cnt_ = 15;
    static constexpr int string_table_cnt_ = 0;
    static constexpr int table_cnt_ = bool_table_cnt_ + double_table_cnt_ + string_table_cnt_;

    enum
    {
        kLED_Green, kLED_Red, kEMS, kSW1, kSW2, kSW3, kPathPlanning,
        kSignalTest, kMotorEnable0, kMotorEnable1, kMotorEnable2, kMotorEnable3,

        kVoltage, kRobotPoseX, kRobotPoseY, kRobotPoseW, kGyro,
        kMotor0, kMotor1, kMotor2, kMotor3, kVelocityX, kVelocityY, kVelocityW,
        kLidarForward, kLidarLeft, kLidarRight
    };
    const std::string table_str_[table_cnt_]
    {
        "LED_Green", "LED_Red", "EMS", "SW1", "SW2", "SW3", "PathPlanning",
        "SignalTest", "MotorEnable0", "MotorEnable1", "MotorEnable2", "MotorEnable3",

        "Voltage", "RobotPoseX", "RobotPoseY", "RobotPoseW", "Gyro",
        "Motor0", "Motor1", "Motor2", "Motor3", "VelocityX", "VelocityY", "VelocityW",
        "LidarForward", "LidarLeft", "LidarRight"
    };

    std::shared_ptr<nt::NetworkTable> table_;
    std::array<nt::NetworkTableEntry, table_cnt_> table_arr_;

    void LaserScanMeasure(int target);
};