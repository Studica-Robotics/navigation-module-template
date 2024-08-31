/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace constant
{
    /**
     * Motors
     */
    static constexpr int TITAN_ID           = 42; // Titan CAN ID
    static constexpr int FRONT_LEFT_MOTOR   = 3;  // M0
    static constexpr int FRONT_RIGHT_MOTOR  = 2;  // M1
    static constexpr int BACK_LEFT_MOTOR    = 1;  // M2
    static constexpr int BACK_RIGHT_MOTOR   = 0;  // M3

    /**
     * Digital I/O
     */
    static constexpr int START_BUTTON       = 11; // Digital Input 11
    static constexpr int E_STOP_BUTTON      = 10; // Digital Input 10
    static constexpr int RUNNING_LED        = 13; // Digital Output 13
    static constexpr int STOPPED_LED        = 14; // Digital Output 14
}
