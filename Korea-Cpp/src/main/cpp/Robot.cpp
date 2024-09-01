#include "Robot.h"
#include "Util.h"

#include "VMX.h"
#include "Lidar.h"
#include "Shuffleboard.h"
#include "PathPlanner.h"

void Robot::StartCompetition()
{
    VMX vmx;
    Lidar lidar;
    Shuffleboard shuffleboard;
    PathPlanner path_planner;

    int past_time;
    while (!m_exit)
    {
        past_time = GetTime();
        
        shuffleboard.Update();
        vmx.Update();

        delay(20, past_time);
    }
}
void Robot::EndCompetition()
{
    m_exit = true;
}

int main()
{
    return frc::StartRobot<Robot>();
}