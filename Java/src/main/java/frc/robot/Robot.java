/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.studica.frc.MockDS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private MockDS ds;
  private boolean active = false;
  private int countLED;
  private boolean prevLEDValue;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    ds = new MockDS();
    RobotContainer.driveTrain.setRunningLED(false);
    RobotContainer.driveTrain.setStoppedLED(false);
    countLED = 1;
    prevLEDValue = true;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // If Start button pushed enabled robot in auto mode
    if (!RobotContainer.driveTrain.getStartButton() && !active)
    {
      ds.enable();
      active = true;
      if (!RobotContainer.driveTrain.scanning)
        RobotContainer.driveTrain.startLidar();
    }
    // If E-Stop button is pushed disable the robot
    if (!RobotContainer.driveTrain.getEStopButton() && active)
    {
      ds.disable();
      active = false;
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() 
  {
    RobotContainer.driveTrain.setRunningLED(true);
    RobotContainer.driveTrain.setStoppedLED(false);
    RobotContainer.driveTrain.stopLidar();
  }

  @Override
  public void disabledPeriodic() 
  {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() 
  {
    RobotContainer.driveTrain.setRunningLED(false);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
    runningLED();
  }

  @Override
  public void teleopInit() 
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.driveTrain.setRunningLED(false);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    runningLED();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void runningLED()
  {
      if ((countLED % 25) == 0)
      {
          if (prevLEDValue)
          {
              RobotContainer.driveTrain.setRunningLED(false);
              prevLEDValue = false;
          }
          else
          {
              RobotContainer.driveTrain.setRunningLED(true);
              prevLEDValue = true;
          }
          countLED = 1;   
      }
      else
      {
          countLED++;
      }
  }
}
