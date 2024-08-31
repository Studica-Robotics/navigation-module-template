#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrain.h"
#include "subsystems/DepthCamera.h"

class AutoCommand : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutoCommand>
{
    public:
        AutoCommand(DriveTrain* drive, DepthCamera* camera);
};