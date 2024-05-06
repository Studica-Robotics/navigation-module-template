package frc.robot.commands.auto;

// import the commands
import frc.robot.commands.driveCommands.Drive;

/**
 * Navigation Command Class
 * <p>
 * This class holds the commands required to finish the navigation task
 */
public class Navigation extends AutoCommand
{
    /**
     * Constructor
     */
    public Navigation()
    {
        /**
         * Calls the drive command and adds a timeout of 10 min. 
         */
        super (new Drive().withTimeout(600.0));
    }
}