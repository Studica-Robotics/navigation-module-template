#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include "Constants.h"

#include "studica/TitanQuad.h"
#include "studica/Lidar.h"
#include "AHRS.h"
#include <math.h>

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
        // Lidar
        bool lidarRunning = true;
    private:
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

        // Holonomic Variables
        double denomonator = 0;
};