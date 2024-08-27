#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include "Constants.h"
#include "numeric"
#include "studica/TitanQuad.h"
#include "studica/Lidar.h"
#include "AHRS.h"

// #include <Pair>
class DriveTrain : public frc2::SubsystemBase
{
    public:
        DriveTrain();
        void Periodic() override;
        double GetYaw(void);
        double GetAngle(void);
        bool GetStartButton(void);
        bool GetEStopButton(void);
        void SetRunningLED(bool on);
        void SetStoppedLED(bool on);
        void ResetYaw(void);
        void LidarStop(void);
        void LidarStart(void);
        void StackMotorControl(double x, double y);
        void TwoWheelMotorControl(double x, double y);
        void SixWheelMotorControl(double x, double y);
        void MecanumMotorControl(double x, double y, double z);
        void XBotMotorControl(double x, double y, double z);
        void HolonomicDrive(double x, double y, double z);
        // Lidar
        bool lidarRunning = true;
                double GetLidarDist(int degree);
        void StopMotor();
        void UpdateLidarData();
        double GetGlobalAngle();
        std::pair<double, double> ArcCompute(double angular_vel, double rotation_radius);

        std::pair<double, double> PolarToCartesian(int angleDegrees);

        studica::TitanQuad frontLeftMotor{constant::TITAN_ID, constant::FRONT_LEFT_MOTOR};
        studica::TitanQuad backLeftMotor{constant::TITAN_ID, constant::BACK_LEFT_MOTOR};
        studica::TitanQuad frontRightMotor{constant::TITAN_ID, constant::FRONT_RIGHT_MOTOR};
        studica::TitanQuad backRightMotor{constant::TITAN_ID, constant::BACK_RIGHT_MOTOR};
        AHRS navX{frc::SPI::Port::kMXP};
        studica::Lidar lidar{studica::Lidar::Port::kUSB1};
        studica::Lidar::ScanData scanData;
        
        frc::DigitalInput startButton{constant::START_BUTTON};
        frc::DigitalInput eStopButton{constant::E_STOP_BUTTON};
        frc::DigitalOutput runningLED{constant::RUNNING_LED};
        frc::DigitalOutput stoppedLED{constant::STOPPED_LED};


        void UpdateReadings();

        double CalculateMean(const std::vector<double> &data);
        double CalculateStdDev(const std::vector<double> &data, double mean);

        void UpdateSensorReadings();
        void UpdateLidarReadings();
        // void UpdateFlags();
        void ProcessLidarData(const std::vector<double>& raw_data, std::vector<double>& processed_data);
        // std::string RobotStateToString(RobotState state);
        bool DetectObstacle(double min_dist, double mean, double stddev);
        double normalCDF(double value, double mean, double stddev);
        double MedianFilter(const std::vector<double>& data, size_t windowSize);

        bool front_obj_flag = false, left_obj_flag = false, right_obj_flag  = false;
        double front_mean_dist, left_mean_dist,right_mean_dist;
        double front_stddev, left_stddev,right_stddev;
    

    private:



        double linear_speed;
        double side_speed;
        double angular_speed;
        int targetAngle;
        std::vector<std::pair<int, double>> data;
        std::vector<double> front_lidar_readings;
        std::vector<double> left_lidar_readings;
        std::vector<double> right_lidar_readings;
        DriveTrain* drive;


        double denomonator = 0;
};