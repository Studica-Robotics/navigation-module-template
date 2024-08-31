/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"
#include "commands/Navigation.h"
RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
//  drive.SetDefaultCommand(Navigation{&drive});


drive.SetDefaultCommand(Navigation{&drive});

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // return &autoCmd;
}
