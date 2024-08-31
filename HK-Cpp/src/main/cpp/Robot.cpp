/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#define DSstop true

void Robot::RobotInit()
{
  m_container.drive.SetRunningLED(false);
  m_container.drive.SetStoppedLED(false);
  active = false;
  countLED = 1;
  prevLEDValue = true;

  // We need to run our vision program in a separate thread.
  // If not, our robot program will not run.
#if defined(__linux__)
  std::thread visionThread(WSHK::_2024::Func::VisionThread);
  visionThread.detach();
#else
  wpi::errs() << "Vision only available on Linux.\n";
  wpi::errs().flush();
#endif
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
#if DSstop
    ds.Enable();
#endif
    active = true;
    if (!m_container.drive.lidarRunning)
    {
      m_container.drive.LidarStart();
    }
  }
  if (!m_container.drive.GetEStopButton() && active)
  {
    active = false;
#if DSstop
    ds.Disable();
#endif
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
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic()
{
  RunningLED();
}

void Robot::TeleopInit()
{
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr)
  {
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
int main()
{
  return frc::StartRobot<Robot>();
}
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
