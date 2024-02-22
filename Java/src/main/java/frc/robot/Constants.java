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

    /**
     * Encoders 
     * Uncomment the correct wheel and comment those not used
     */

    /** Omni Wheel Radius */
    private static final double wheelRadius         = 51.0; //mm

    /** Mecanum Wheel Radius */
    // private static final double wheelRadius      = 50.0; //mm

    /** All-Terrain Wheel Radius */
    // private static final double wheelRadius      = 62.5; //mm

    /** Drive Wheel Radius */
    // private static final double wheelRadius      = 50.0; //mm

    /** Encoder pulse per revolution */
    private static final double pulsePerRevolution  = 1464.0;

    /** Gear Ratio between encoder and wheel */
    private static final double gearRatio           = 1.0 / 1.0;

    /** Pulse per revolution of the wheel */
    private static final double wheelPulseRatio     = pulsePerRevolution * gearRatio;

    /** Distance per tick */
    public static final double WHEEL_DIST_PER_TICK  = (Math.PI * 2.0 * wheelRadius) / wheelPulseRatio;

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
