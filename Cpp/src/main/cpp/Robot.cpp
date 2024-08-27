/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>




void Robot::VisionThread() {
    std::pair<double, double> coord;
    cv::Point cloud_points;

    camera = camera_server->StartAutomaticCapture();

    camera.SetResolution(640, 640);

    // Setup a CvSource. This will send images back to the Dashboard
    outputStream = camera_server->PutVideo("blankImage", 640, 640);
    
    tab.Add("Camera", outputStream).WithWidget(frc::BuiltInWidgets::kCameraStream)
                                  .WithProperties({{"Show crosshair", nt::Value::MakeBoolean(false)},
                                                   {"Show controls", nt::Value::MakeBoolean(false)}
                                                  });

    // Mats are very memory expensive. Lets reuse this Mat.
    while (true) {
        cv::Mat blankImage(480, 640, CV_8UC3, cv::Scalar(255, 255, 255)); // Create a white image

        // Draw a coordinate grid for better visualization
        for (int i = 0; i < blankImage.rows; i += 40) {
            cv::line(blankImage, cv::Point(0, i), cv::Point(blankImage.cols, i), cv::Scalar(200, 200, 200));
        }
        for (int i = 0; i < blankImage.cols; i += 40) {
            cv::line(blankImage, cv::Point(i, 0), cv::Point(i, blankImage.rows), cv::Scalar(200, 200, 200));
        }

        // Plot the LiDAR position as a black dot at the center of the image
        cv::circle(blankImage, cv::Point(blankImage.cols / 2, blankImage.rows / 2), 5, cv::Scalar(0, 0, 0), -1);

        // Plot the LiDAR data points
        for (int j = 0; j < 360 ; j++) {
            coord = m_container.drive.PolarToCartesian(j);
            cloud_points = ConvertToPixelCoordinates(coord.first, coord.second, 640, 480, 2);
            cv::circle(blankImage, cloud_points, 2, cv::Scalar(0, 0, 255), 2);
        }
        outputStream.PutFrame(blankImage);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

cv::Point Robot::ConvertToPixelCoordinates(double x, double y, int imageWidth, int imageHeight, double maxRangeMeters) {
    double pixelsPerMeterX = imageWidth / (2.0 * maxRangeMeters);
    double pixelsPerMeterY = imageHeight / (2.0 * maxRangeMeters);

    int pixelX = static_cast<int>((x * pixelsPerMeterX) + (imageWidth / 2.0));
    int pixelY = static_cast<int>((y * pixelsPerMeterY) + (imageHeight / 2.0));

    return cv::Point(pixelX, pixelY);
}

void Robot::RobotInit() 
{
  std::thread vision_thread{&Robot::VisionThread,this};
  vision_thread.detach();

  m_container.drive.SetRunningLED(false);
  m_container.drive.SetStoppedLED(false);
  active = false;
  countLED = 1;
  m_container.drive.LidarStop();
  
  prevLEDValue = true;
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{ 

  frc2::CommandScheduler::GetInstance().Run(); 


  if (!m_container.drive.GetStartButton() && !active)
  {
    ds.Enable();


    active = true;
    if (!m_container.drive.lidarRunning)
    {
      m_container.drive.LidarStart();
    }
  }
  if (!m_container.drive.GetEStopButton() && active)
  {


    active = false;
    
    ds.Disable();
  }
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() 
{

  m_container.drive.SetRunningLED(false);
  m_container.drive.SetStoppedLED(true);
  if (m_container.drive.lidarRunning)
  {
    m_container.drive.LidarStop();
  }
}

void Robot::DisabledPeriodic() 
{
  // frc::SmartDashboard::PutString("Robot Status", "Disabled Periodic");
  //   if (m_container.drive.lidarRunning)
  // {
  //   m_container.drive.LidarStop();
  // }
  m_container.drive.SetStoppedLED(true);
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() 
{

  m_container.drive.SetRunningLED(true);
  m_container.drive.SetStoppedLED(false);
  // m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() 
{

  RunningLED(); 
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  m_container.drive.SetRunningLED(true);
  m_container.drive.SetStoppedLED(false);
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() 
{

  RunningLED();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

void Robot::RunningLED()
{
  if ((countLED % 25) == 0)
  {
    if (prevLEDValue)
    {
      m_container.drive.SetRunningLED(false);
      prevLEDValue = false;
    }
    else
    {
      m_container.drive.SetRunningLED(true);
      prevLEDValue = true;
    }
    countLED = 1;
  }
  else
  {
    countLED++;
  }
}
