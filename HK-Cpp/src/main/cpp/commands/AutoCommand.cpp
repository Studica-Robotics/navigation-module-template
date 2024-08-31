#include "commands/AutoCommand.h"
#include "commands/Navigation.h"
#include <frc2/command/ParallelRaceGroup.h>

AutoCommand::AutoCommand(WSHK::_2024::DriveTrain* drive)
{
    AddCommands
    (
        Navigation(drive).WithTimeout(3600.0_s) // Timeout after 60 min
    );
}
