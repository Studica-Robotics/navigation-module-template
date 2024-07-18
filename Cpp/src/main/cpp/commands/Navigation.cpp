#include "commands/Navigation.h"

Navigation::Navigation(DriveTrain* driveTrain, DepthCamera* depthCamera) : drive{driveTrain}, camera{depthCamera}
{
    AddRequirements({driveTrain, depthCamera});
}

void Navigation::Initialize()
{
    drive->ResetYaw();
}

void Navigation::Execute()
{
    drive->StackMotorControl(0.5, 0.0); // Move forward at 50%
}

void Navigation::End(bool interrupted)
{
    drive->StackMotorControl(0.0, 0.0); // Stop
}

bool Navigation::IsFinished()
{
    return false;
}