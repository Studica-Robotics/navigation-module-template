package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DepthCamera;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase
{
    // Grab the subsystem instance from RobotContainer
    private static final DriveTrain drive = RobotContainer.driveTrain;
    private static final DepthCamera depthCamera = RobotContainer.depthCamera;

    /**
     * Constructor
     */
    public Drive ()
    {
        addRequirements(drive, depthCamera);
    }

    /**
     * Runs once before execute
     */
    @Override
    public void initialize()
    {
        drive.zeroYaw();
    }

    /**
     * Called continously until command is ended
     */
    @Override
    public void execute()
    {
        drive.stackMotorControl(0.5, 0.0); // Move robot forward at 50%
    }

    /**
     * Called when the command is told to end
     */
    @Override
    public void end (boolean interrupted)
    {
        drive.stackMotorControl(0.0, 0.0); // Ensure motors are stopped
    }

    /**
     * Creates the end condition for the command
     */
    @Override
    public boolean isFinished()
    {
        return false; // We dont want it to end
    }
}