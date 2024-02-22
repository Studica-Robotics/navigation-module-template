package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.Lidar;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

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
     * Encoders
     */
    private TitanQuadEncoder frontLeftEncoder;
    private TitanQuadEncoder backLeftEncoder;
    private TitanQuadEncoder frontRightEncoder;
    private TitanQuadEncoder backRightEncoder;

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
         * Encoders
         */

        frontLeftEncoder = new TitanQuadEncoder(frontLeftMotor, Constants.FRONT_LEFT_MOTOR, Constants.WHEEL_DIST_PER_TICK);
        backLeftEncoder = new TitanQuadEncoder(backLeftMotor, Constants.BACK_LEFT_MOTOR, Constants.WHEEL_DIST_PER_TICK);
        frontRightEncoder = new TitanQuadEncoder(frontRightMotor, Constants.FRONT_RIGHT_MOTOR, Constants.WHEEL_DIST_PER_TICK);
        backRightEncoder = new TitanQuadEncoder(backRightMotor, Constants.BACK_RIGHT_MOTOR, Constants.WHEEL_DIST_PER_TICK);

        /**
         * Motor Flags
         */
        
        // Front Left Motor
        frontLeftMotor.setInverted(false);
        frontLeftMotor.invertRPM();
        frontLeftEncoder.setReverseDirection();
 
        // Back Left Motor
        backLeftMotor.setInverted(false);
        backLeftMotor.invertRPM();
        backLeftEncoder.setReverseDirection();

        // Front Right Motor
        frontRightMotor.setInverted(false);
        frontRightMotor.invertRPM();
        frontRightEncoder.setReverseDirection();

        // Back Right Motor
        backRightMotor.setInverted(false);
        backRightMotor.invertRPM();
        backRightEncoder.setReverseDirection();

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
    }

    /**
     * Reset all encoders
     */
    public void resetEncoders()
    {
        frontLeftEncoder.reset();
        backLeftEncoder.reset();
        frontRightEncoder.reset();
        backRightEncoder.reset();
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
     * Get the distance travelled by the front left motor
     * @return - the distance in mm travelled.
     */
    public double getFrontLeftEncoder()
    {
        return frontLeftEncoder.getEncoderDistance();
    }

    /**
     * Get the distance travelled by the back left motor
     * @return - the distance in mm travelled.
     */
    public double getBackLeftEncoder()
    {
        return backLeftEncoder.getEncoderDistance();
    }

    /**
     * Get the distance travelled by the front right motor
     * @return - the distance in mm travelled.
     */
    public double getFrontRightEncoder()
    {
        return frontRightEncoder.getEncoderDistance();
    }

    /**
     * Get the distance travelled by the back right motor
     * @return - the distance in mm travelled.
     */
    public double getBackRightEncoder()
    {
        return backRightEncoder.getEncoderDistance();
    }

    /**
     * Get the RPM of the Front Left Motor
     * @return - RPM of Front Left Motor
     */
    public double getFrontLeftRPM()
    {
        return frontLeftMotor.getRPM();
    }

    /**
     * Get the RPM of the Back Left Motor
     * @return - RPM of Back Left Motor
     */
    public double getBackLeftRPM()
    {
        return backLeftMotor.getRPM();
    }

    /**
     * Get the RPM of the Front Right Motor
     * @return - RPM of Front Right Motor
     */
    public double getFrontRightRPM()
    {
        return frontRightMotor.getRPM();
    }

    /**
     * Get the RPM of the Back Right Motor
     * @return - RPM of Back Right Motor
     */
    public double getBackRightRPM()
    {
        return backRightMotor.getRPM();
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
            SmartDashboard.putNumber("Front Left Encoder Dist", getFrontLeftEncoder());
            SmartDashboard.putNumber("Back Left Encoder Dist", getBackLeftEncoder());
            SmartDashboard.putNumber("Front Right Encoder Dist", getFrontRightEncoder());
            SmartDashboard.putNumber("Back Right Encoder Dist", getBackRightEncoder());
            SmartDashboard.putNumber("Front Left RPM", getFrontLeftRPM());
            SmartDashboard.putNumber("Back Left RPM", getBackLeftRPM());
            SmartDashboard.putNumber("Front Right RPM", getFrontRightRPM());
            SmartDashboard.putNumber("Back Right RPM", getBackRightRPM());
        }
    }
}