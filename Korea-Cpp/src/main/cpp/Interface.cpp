#include "Interface.h"

namespace Interface
{
    bool EMS, SW1, SW2, SW3;
    bool SW1_Pressed, SW2_Pressed, SW3_Pressed;
    bool LED_G, LED_R;
    
    std::vector<LaserScan> laser_scan(450), laser_scan_storage(450);
    bool laser_scan_flg, laser_scan_flg2;
    
    double GyroYaw;
    bool GyroReset;
    Vector3d RobotPose;
    double Motor[4], Velocity[3];

    bool PathPlanning, SignalTest;

    bool Autonomous;
}