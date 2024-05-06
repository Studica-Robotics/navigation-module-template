/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants 
{
    /**
     * Titan CAN ID
     */
    public static final int TITAN_ID                = 42;

    /* Drive Base */

    /**
     * Front Left Motor
     */
    public static final int FRONT_LEFT_MOTOR        = 0;

    /**
     * Front Right Motor
     */
    public static final int FRONT_RIGHT_MOTOR       = 1;

    /**
     * Back Left Motor
     */
    public static final int BACK_LEFT_MOTOR         = 2;

    /**
     * Back Right Motor
     */
    public static final int BACK_RIGHT_MOTOR        = 3;

    /* Digital I/O */

    /** Start Button */
    public static final int START_BUTTON            = 9; // Digital input 9

    /** E-STOP */
    public static final int E_STOP_SWITCH           = 11; // Digital input 11;

    /** Running LED */
    public static final int RUNNING_LED             = 20; // Digital Output 20

    /** Stopped LED */
    public static final int STOPPED_LED             = 21; // Digital Output 21
}
