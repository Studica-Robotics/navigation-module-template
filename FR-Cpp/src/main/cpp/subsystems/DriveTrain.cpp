#include "subsystems/DriveTrain.h"

#define DEBUG true

DriveTrain::DriveTrain()
{
    ResetYaw();
    //Motor Invert flags
    frontLeftMotor.SetInverted(false);
    backLeftMotor.SetInverted(false);
    frontRightMotor.SetInverted(false);
    backRightMotor.SetInverted(false);
}

double DriveTrain::GetYaw()
{
    return navX.GetYaw();
}

double DriveTrain::GetAngle()
{
    return navX.GetAngle();
}

void DriveTrain::ResetYaw()
{
    navX.ZeroYaw();
}

bool DriveTrain::GetStartButton()
{
    return startButton.Get();
}

bool DriveTrain::GetEStopButton()
{
    return eStopButton.Get();
}

void DriveTrain::SetRunningLED(bool on)
{
    runningLED.Set(on);
}

void DriveTrain::SetStoppedLED(bool on)
{
    stoppedLED.Set(on);
}

void DriveTrain::LidarStop()
{
    lidar.Start();
    
    //lidar.ClusterConfig(50.0f, 5);
    lidar.KalmanConfig(1e-4f, 0.2f, 1.0f);
    // lidar.MovingAverageConfig(5);
    //lidar.MedianConfig(5);
    // lidar.JitterConfig(50.0f);
    // enable filter
    lidar.EnableFilter(studica::Lidar::Filter::kKALMAN, true);

    if (!lidarRunning)
    {
        lidarRunning = true;
    }
}

void DriveTrain::LidarStart()
{
    if (lidarRunning)
    {
        lidar.Stop();
        lidarRunning = false;
    }
}

void DriveTrain::StackMotorControl(double x, double y)
{
    frontLeftMotor.Set(y + x);
    backLeftMotor.Set(y + x);
    frontRightMotor.Set(y - x);
    backRightMotor.Set(y - x);
}

void DriveTrain::TwoWheelMotorControl(double x, double y)
{
    frontLeftMotor.Set(y + x);
    frontRightMotor.Set(y - x);
}

void DriveTrain::SixWheelMotorControl(double x, double y)
{
    frontLeftMotor.Set(y + x);
    backLeftMotor.Set(y + x);
    frontRightMotor.Set(y - x);
    backRightMotor.Set(y - x);
}

void DriveTrain::MecanumMotorControl(double x, double y, double z)
{
    denomonator = fmax(fabs(y) + fabs(x) + fabs(z), 1.0);
    frontLeftMotor.Set(y + (x) + z / denomonator);
    backLeftMotor.Set(y - (x) + z / denomonator);
    frontRightMotor.Set(y - (x) - z / denomonator);
    backRightMotor.Set(y + (x) - z / denomonator);
}

void DriveTrain::XBotMotorControl(double x, double y, double z)
{
    denomonator = fmax(fabs(y) + fabs(x) + fabs(z), 1.0);
    frontLeftMotor.Set(y + (x) + z / denomonator);
    backLeftMotor.Set(y - (x) + z / denomonator);
    frontRightMotor.Set(y - (x) - z / denomonator);
    backRightMotor.Set(y + (x) - z / denomonator);
}

void DriveTrain::Periodic()
{
    if (lidarRunning)
    {
        scanData = lidar.GetData(); // Update scan data struct
    }
    #if DEBUG
        frc::SmartDashboard::PutNumber("Yaw", GetYaw());
        frc::SmartDashboard::PutNumber("Angle", GetAngle());
    #endif
}