package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.Lidar;
import com.studica.frc.TitanQuad;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase
{
    // Debug Flag
    private boolean debug = false;

    /**
     * Motors
     */
    private TitanQuad frontLeftMotor;
    private TitanQuad backLeftMotor;
    private TitanQuad frontRightMotor;
    private TitanQuad backRightMotor;

    /**
     * IMU
     */
    private AHRS navX;

    /**
     * Digital I/O
     */
    private DigitalInput startBtn;
    private DigitalInput eStopBtn;
    private DigitalOutput runningLED;
    private DigitalOutput stoppedLED;

    /**
     * Lidar
     */
    private Lidar lidar;
    private Lidar.ScanData scanData;
    public boolean scanning = true; // flag to prevent updating when not scanning

    /**
     * Variables to reduce overhead
     */
    double denomantor = 0;

    /**
     * DriveTrain Constructor
     */
    public DriveTrain()
    {
        /**
         * Motors
         */

        frontLeftMotor = new TitanQuad(Constants.TITAN_ID, Constants.FRONT_LEFT_MOTOR);
        backLeftMotor = new TitanQuad(Constants.TITAN_ID, Constants.BACK_LEFT_MOTOR);
        frontRightMotor = new TitanQuad(Constants.TITAN_ID, Constants.FRONT_RIGHT_MOTOR);
        backRightMotor = new TitanQuad(Constants.TITAN_ID, Constants.BACK_RIGHT_MOTOR);

        Timer.delay(1.0); // Wait 1s for Titan to configure

        /**
         * Motor Flags
         */
        
        // Front Left Motor
        frontLeftMotor.setInverted(false);

        // Back Left Motor
        backLeftMotor.setInverted(false);

        // Front Right Motor
        frontRightMotor.setInverted(false);

        // Back Right Motor
        backRightMotor.setInverted(false);

        /**
         * IMU
         */
        navX = new AHRS(SPI.Port.kMXP);

        /**
         * Digital I/O
         */
        startBtn = new DigitalInput(Constants.START_BUTTON);
        eStopBtn = new DigitalInput(Constants.E_STOP_SWITCH);
        runningLED = new DigitalOutput(Constants.RUNNING_LED);
        stoppedLED = new DigitalOutput(Constants.STOPPED_LED);

        /**
         * Lidar
         */
        lidar = new Lidar(Lidar.Port.kUSB1);

        /**
         * Cleanup functions after init
         */
        zeroYaw();
    }

    /**
     * Reset Yaw
     */
    public void resetYaw()
    {
        navX.zeroYaw();
    }

    /**
     * Turn the Running LED on or off
     * @param on - set to true to turn on the LED
     */
    public void setRunningLED(Boolean on)
    {
        runningLED.set(on);
    }

    /**
     * Turn the stopped LED on or off
     * @param on - set to true to turn on the LED
     */
    public void setStoppedLED(Boolean on)
    {
        stoppedLED.set(on);
    }

    /**
     * Start the lidar if it is stopped.
     */
    public void startLidar()
    {
        lidar.start();
        scanning = true;
    }

    /**
     * Stop the lidar if required. This will ease the load on CPU.
     */
    public void stopLidar()
    {
        lidar.stop();
        scanning = false;
    }

    /**
     * Get the value of the start button
     * @return - the current logic of the start button. If wired correctly this will be active low.
     */
    public boolean getStartButton()
    {
        return startBtn.get();
    }

    /**
     * Get the value of the e-stop switch
     * @return - the current logic of the e-stop switch. If wired correctly this will be active low.
     */
    public boolean getEStopButton()
    {
        return eStopBtn.get();
    }

    /**
     * Controls the stack robot
     * @param x - movement in the x axis
     * @param y - movement in the y axis
     */
    public void stackMotorControl(double x, double y)
    {
        frontLeftMotor.set(y + x);
        backLeftMotor.set(y + x);
        frontRightMotor.set(y - x);
        backRightMotor.set(y - x);
    }

    /**
     * Controls the two wheel robot
     * @param x - movement in the x axis
     * @param y - movement in the y axis
     */
    public void twoWheelMotorControl(double x, double y)
    {
        frontLeftMotor.set(y + x);
        frontRightMotor.set(y - x);
    }

    /**
     * Controls the six wheel robot
     * @param x - movement in the x axis
     * @param y - movement in the y axis
     */
    public void sixWheel(double x, double y)
    {
        frontLeftMotor.set(y + x);
        backLeftMotor.set(y + x);
        frontRightMotor.set(y - x);
        backRightMotor.set(y - x);
    }

    /**
     * Controls the mecanum robot
     * @param x - movement in the x axis
     * @param y - movement in the y axis
     * @param z - movement in the z axis
     */
    public void mecanumMotorControl(double x, double y, double z)
    {
        denomantor = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(z), 1.0);
        frontLeftMotor.set(y + (x) + z / denomantor);
        backLeftMotor.set(y - (x) + z / denomantor);
        frontRightMotor.set(y - (x) - z / denomantor);
        backRightMotor.set(y + (x) - z / denomantor);
    }

    /**
     * Controls the x bot robot
     * @param x - movement in the x axis
     * @param y - movement in the y axis
     * @param z - movement in the z axis
     */
    public void xBotMotorControl(double x, double y, double z)
    {
        denomantor = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(z), 1.0);
        frontLeftMotor.set(y + (x) + z / denomantor);
        backLeftMotor.set(y - (x) + z / denomantor);
        frontRightMotor.set(y - (x) - z / denomantor);
        backRightMotor.set(y + (x) - z / denomantor);
    }

    /**
     * Get the yaw value from the IMU
     * @return - ± 180°
     */
    public double getYaw()
    {
        return navX.getYaw();
    }

    /**
     * Get the continuous angle from the IMU
     * @return - the continuous angle from the IMU
     */
    public double getAngle()
    {
        return navX.getAngle();
    }

    /**
     * Zero the yaw of the internal IMU.
     */
    public void zeroYaw()
    {
        navX.zeroYaw();
    }

    /**
     * Everything in this method runs once every robot loop a.k.a 20ms
     */
    @Override
    public void periodic()
    {
        // We want this to run every loop so the lidar data is always the most recent
        if (scanning)
        {
            //Update scanData class
            scanData = lidar.getData();
        }

        // Only output data to the dashboard for debugging purposes as it's a large overhead
        if (debug)
        {
            SmartDashboard.putNumber("Yaw", getYaw());
            SmartDashboard.putNumber("Angle", getAngle());
        }
    }
}