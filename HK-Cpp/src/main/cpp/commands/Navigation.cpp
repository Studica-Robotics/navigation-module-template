#include "commands/Navigation.h"


Navigation::Navigation(WSHK::_2024::DriveTrain* driveTrain) : drive{driveTrain}
{
    AddRequirements({driveTrain});
}

void Navigation::Initialize()
{
    drive->ResetYaw();
}

void Navigation::Execute()
{
    // drive->SetRobot(WSHK::_2024::DriveTrain::RobotDrive::X_Drive);
}

void Navigation::End(bool interrupted)
{
    // Stop
}

bool Navigation::IsFinished()
{
    return false;
}