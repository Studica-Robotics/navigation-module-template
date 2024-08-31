#include "commands/AutoCommand.h"
#include "commands/Navigation.h"
#include <frc2/command/ParallelRaceGroup.h>

AutoCommand::AutoCommand(DriveTrain* drive, DepthCamera* camera)
{
    AddCommands
    (
        Navigation(drive, camera).WithTimeout(600.0_s) // Timeout after 10 min
    );
}