#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>

#include "Painel.hpp"
#include "MovementControl.hpp"

class Robot : public frc::TimedRobot 
{
  public:

  PAINEL painel;
  MovementControl move;
  
  void RobotInit() override {}
  void TestInit() override {}
  void TestPeriodic() override {}
  void TeleopInit() override {}
  void TeleopPeriodic() override {}

  void AutonomousInit() override 
  {
    move.initMove();
  }

  bool iniciar = false;
  void AutonomousPeriodic() override
  {
    bool bStart = painel.Start();

    if(bStart)
    {
      iniciar = true;
    }

    if(iniciar)
    {
      move.moveByLidar();
    }
      
  }

  private:

};

#ifndef RUNNING_FRC_TESTS
int main() 
{ 
  return frc::StartRobot<Robot>();  
}
#endif