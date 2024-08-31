#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrain.h"
#include "subsystems/DepthCamera.h"
#include "RobotContainer.h"

class Navigation : public frc2::CommandHelper<frc2::CommandBase, Navigation>
{
    public:
        Navigation(DriveTrain* drive, DepthCamera* camera);
        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        DriveTrain* drive;
        DepthCamera* camera;
};