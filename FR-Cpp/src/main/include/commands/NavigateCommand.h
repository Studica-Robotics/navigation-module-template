#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "subsystems/DriveTrain.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>
#include <numeric>
#include <chrono>

class NavigateCommand : public frc2::CommandHelper<frc2::CommandBase, NavigateCommand>
{
private:
    enum class MotorControlType
    {
        STACK,
        TWOWHEELS,
        SIXWHEELS,
        MECANUM,
        XBOT,
        UNKNOWN
    };
    //--------------------------------------
private:
    const float maxLinearVelocity = 0.2f;         //linear speed
    const float maxAngularVelocity = 0.4f;        //angular speed
    const int lidarOffset = 180;                  //offset of lidar from the robot front. (Gray motor is the front of the lidar)
    MotorControlType motorControltype;

    // -------------------------------------- Studica Stack
public:
    explicit NavigateCommand(DriveTrain *drive);
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

    // -------------------------------------- Utilities
private:
    void SendControl(float linearSpeed, float angularSpeed);
    int NormalizeAngle(float angle);
    int AngleToIndex(int angle);
    float GetAverageDistanceInZone(int startAngle, int endAngle);
    int AdjustAngleForLidarOffset(int angle);
    bool IsAngleInRange(int angle, int startAngle, int endAngle);
    float CalculateMedianDistance(int angleStart, int angleEnd);
    float CalculateAverageDistance(int angleStart, int angleEnd);
    void DebugLidar(int step);

    // -------------------------------------- ALGO1
private:
    void FollowingHoles();
    bool CheckObstacleInRange(int angleStart, int angleEnd);
    bool CheckEndConditionNoEndWall(int angleStart, int angleEnd);
    bool CheckEndCondition();
    float DetermineTurnDirection();
    void TurnInDirection(float direction);

    const int angleFront = 0;                     //should be 0
    const int angleRight = 40;                    //angle where front right corner of the robot is in the colision cercle
    const int angleLeft = 320;                    //angle where front left corner of the robot is in the colision cercle
    const int angleFrontLeft = 270;               //left angle constrain
    const int angleFrontRight = 90;               //right angle constrain
    const float obstacleThreshold = 275;          //min distance for an angle to be an obstacle.

    const int angleEndLeft = 270;                 //left angle where robot check if no wall for end
    const int angleEndRight = 90;                 //right angle where robot check if no wall for end
    const float endNoObstacleDistance = 1500;      //distance which says that robot finished the board (if no wall behind stop)
    const int endNbAnglesTolerance = 5;           //nb of angles that can be under the endNoObstacleDistance in case of rigged values

    const float endMinFarSideThreshold = 400;     //min distance that the robot should be compared to the opposite end side wall
    const float endFarSideThreshold = 500;        //max distance that the robot should be compared to the opposite end side wall
    const float endCloseSideThreshold = 200;      //distance that the robot should be compared to the close end side wall
    const float endFrontSideThreshold = 200;      //distance that the robot should be compared to the front end wall
    const float endRotTolerance = 5.0f;           //tolerance for end rotation (0)

    bool end = false;
    int obstacleId = -1;
    bool isCalcul = false;
    float turnDirection = 0.0f;
    float robotYaw;

    // -------------------------------------- ALGO2
private:
    enum class FollowWallState { None, MovingForward, Turning, End, Stop };
    FollowWallState currentState = FollowWallState::None;

    struct Waypoint
    {
        int TargetYaw;
        float WallThreshold;
        int AngleToCheck;
    };

    std::vector<Waypoint> waypoints;
    Waypoint endWaypoint;
    int currentStep = 0;
    void WaypointsInit();
    void FollowWall();
    void Forward();
    void Turn(int targetYaw);

    // -------------------------------------- ALGO3
private:
    float targetMovementDistance = 0.42f;
    const float minimumFrontWallDistance = 0.45f;    // minimum distance before considering a wall in front
    const float sideWallDistance = 0.5f;            // distance from the side wall
    const int leftWallAngle = 260;                  // angle to detect the left wall
    const int rightWallAngle = 80;                  // angle to detect the right wall
    const int frontRightAngle = 30;                 // angle to detect the front right
    const int frontLeftAngle = 330;                 // angle to detect the front left
    const int lidarAngleRange = 3;                  // lidar measurement angle range
    const float startMovementDistance = 300;        // distance to move to start the algorithm
    const float distanceToMoveWhenWallLost = 250;   // distance to move when the wall is lost
    const float distanceToCrossLostWall = 500;      // distance to cross when the wall is lost
    const float wallEndThreshold = 1.5f;            // threshold to determine the end of a wall

        // ----------------------
    enum class WallFollowingState { Idle, Entering, MovingToWall, PrecisionMovement, FollowingWall, Turning, Finished, Stopped };
    WallFollowingState currentWallFollowingState = WallFollowingState::Idle;

    float initialFrontWallDistance = 0.0f;
    int targetRotationYaw = 0;
    bool isInitialMovement = true;
    bool hasLostLeftWall = false;
    bool hasLostLeftWallAgain = false;
    float lastAngularVel = 0.0f;
        // ----------------------
    void WallFollowing();
    void MoveForwardWithoutCorrection();
    void MoveForwardWithCorrection();
    void RotateToYaw(int targetYaw);
    int CalculateWallAngle(float frontDistance, float sideDistance, float angleBetween);
    int ConvertAngleToLidarIndex(int angle);
    int NormalizeLidarAngle(int angle);

 // -------------------------------------- ALGO4
private:
    const float close_threshold = 290.0f; 
    const float cul_de_sac_threshold = 350.0f; 

    const int lidar_start_angle = 280; 
    const int lidar_end_angle = 80; 
    const int lidar_angle_range = 10; 
    const int total_zones = 16; 
    const int group_size = 4; 
    const int total_groups = 4; 

    const float threshold = 300.0f; 

    int front_left_angle = 290;
    int front_right_angle = 50;

    void Following();
    float CalculateAngularVelocity(const std::vector<float>& zoneDistances);
    bool finish;
    bool starting = true;

    float front_left_distance = 0;
    float front_right_distance = 0;

private:
    std::ofstream logFile;
    DriveTrain *m_drive;
    studica::Lidar::ScanData scanData;
};
