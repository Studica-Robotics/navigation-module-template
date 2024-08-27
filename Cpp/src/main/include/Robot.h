/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"
#include "studica/MockDS.h"

#include <string>

#include <cstdio>
#include <thread>

#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override; 
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 void VisionThread();
  cv::Point ConvertToPixelCoordinates(double x, double y, int imageWidth, int imageHeight, double maxRangeMeters);

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

  RobotContainer m_container;
  studica::MockDS ds;
  bool active;
  int countLED;
  bool prevLEDValue;
  void RunningLED();

  
  frc::CameraServer *camera_server = frc::CameraServer::GetInstance();
  cs::UsbCamera camera;
  // cs::CvSink cvSink;
  cs::CvSource outputStream;
  // cv::Mat mat;
  // cv::Mat threshold;
  frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("Telemetry");
};
