#include "commands/NavigateCommand.h"

NavigateCommand::NavigateCommand(DriveTrain *drive) : m_drive(drive)
{
    AddRequirements({drive});
}

/**********************STUDICA STACK*****************************************/
void NavigateCommand::Initialize()
{
    m_drive->ResetYaw();
    motorControltype = NavigateCommand::MotorControlType::STACK;
}

void NavigateCommand::Execute()
{
    robotYaw = NormalizeAngle(m_drive->GetYaw());
}

void NavigateCommand::End(bool interrupted)
{
    end = true;
    SendControl(0.0, 0.0);
}

bool NavigateCommand::IsFinished()
{
    return end;
}

/***********************************UTITILIES**************************************/

void NavigateCommand::SendControl(float linearSpeed, float angularSpeed)
{
    switch(motorControltype)
    {
        case MotorControlType::STACK:
            m_drive->StackMotorControl(linearSpeed, angularSpeed);
            break;
        case MotorControlType::TWOWHEELS:
            m_drive->TwoWheelMotorControl(linearSpeed, angularSpeed);
            break;
        case MotorControlType::SIXWHEELS:
            m_drive->SixWheelMotorControl(linearSpeed, angularSpeed);
            break;
        case MotorControlType::MECANUM:
            m_drive->MecanumMotorControl(0.0, linearSpeed, angularSpeed);
            break;
        case MotorControlType::XBOT:
            m_drive->XBotMotorControl(0.0, linearSpeed, angularSpeed);
            break;
        default:
            throw std::invalid_argument("unkown motor control type");
    }
}

void NavigateCommand::DebugLidar(int step)
{
    logFile.open("/home/pi/lidar.log");
    for (int i = 0; i < 360; i += step)
    {
        logFile << "distance at angle #" << i << ": " << m_drive->lidar.GetData().distance[i] << std::endl;
    }
    logFile.close();
}

int NavigateCommand::NormalizeAngle(float angle)
{
    int index = (static_cast<int>(angle) + 360) % 360;
    return index;
}

int NavigateCommand::AngleToIndex(int angle)
{
    int index = (angle - lidarOffset + 360) % 360;
    return index;
}

float NavigateCommand::GetAverageDistanceInZone(int startAngle, int endAngle)
{
    float sumOfDistances = 0;
    int count = 0;

    auto accumulateDistances = [&](int angle) {
        int index = AngleToIndex(angle);
        float distance = m_drive->lidar.GetData().distance[index];
        if (distance > 1)
        {
            sumOfDistances += distance;
            count++;
        }
    };

    if (startAngle <= endAngle)
    {
        for (int angle = startAngle; angle <= endAngle; ++angle)
        {
            accumulateDistances(angle);
        }
    }
    else
    {
        for (int angle = startAngle; angle < 360; ++angle)
        {
            accumulateDistances(angle);
        }
        for (int angle = 0; angle <= endAngle; ++angle)
        {
            accumulateDistances(angle);
        }
    }

    return count == 0 ? 0.0f : sumOfDistances / count;
}

float NavigateCommand::CalculateMedianDistance(int angleStart, int angleEnd)
{
    std::vector<float> distances;

    auto accumulateDistances = [&](int angle) {
        int index = AngleToIndex(angle);
        float distance = m_drive->lidar.GetData().distance[index];
        if (distance > 1)
        {
            distances.push_back(distance);
        }
    };

    if (angleStart <= angleEnd)
    {
        for (int angle = angleStart; angle <= angleEnd; ++angle)
        {
            accumulateDistances(angle);
        }
    }
    else
    {
        for (int angle = angleStart; angle < 360; ++angle)
        {
            accumulateDistances(angle);
        }
        for (int angle = 0; angle <= angleEnd; ++angle)
        {
            accumulateDistances(angle);
        }
    }

    if (distances.empty())
    {
        return 0.0f;
    }

    std::sort(distances.begin(), distances.end());
    size_t size = distances.size();
    float median = 0;
    if (size % 2 == 0)
        median = (distances[size / 2 - 1] + distances[size / 2]) / 2.0f;
    else
        median = distances[size / 2];

    return median;
}

/**************************ALGO1************************************/

void NavigateCommand::FollowingHoles()
{
    if (end)
        return;

    m_drive->scanData = m_drive->lidar.GetData();

    // return;

    //float robotYaw = NormalizeAngle(m_drive->GetYaw());
    // if (CheckEndConditionNoEndWall(angleLeft, angleRight))
    // {

    //     SendControl(0, 0);
    //     end = true;
    //     return;

    //     if (robotYaw > angleEndLeft || robotYaw < angleEndRight)
    //     {
    //         SendControl(0, 0);
    //         end = true;
    //         return;
    //     }
    // }

    if (!CheckObstacleInRange(angleLeft, angleRight))
    {
        obstacleId = -1;
        turnDirection = 0.0f;
        SendControl(maxLinearVelocity, 0.0);
        return;
    }

    if (turnDirection == 0.0f)
        turnDirection = DetermineTurnDirection();
    TurnInDirection(turnDirection);

    logFile << turnDirection << " : " << obstacleId << std::endl;
}

bool NavigateCommand::CheckObstacleInRange(int angleStart, int angleEnd)
{
    bool obstacleDetected = false;
    std::vector<int> ids;
    bool isBlind = true;

    int startIndex = AngleToIndex((angleStart));
    int endIndex = AngleToIndex((angleEnd));

    for (int i = startIndex; i != endIndex; i = (i + 1) % 360)
    {
        float dist = m_drive->lidar.GetData().distance[i];
        if (dist > 1) // check if at least one angle has a true value
            isBlind = false;

        if (dist < obstacleThreshold && dist >= 1)
        {
            ids.push_back(i);
            obstacleDetected = true;
            obstacleId = i;
        }
    }

    if (ids.size() >= 2)
    {
        obstacleId = (ids.at(0) + ids.back()) / 2; // get the middle angle between the lowest and highest angle detected
    }

    if (isBlind) // if every values is rigged, assume there's an obstacle
    {
        obstacleDetected = true;
        obstacleId = NormalizeAngle(lidarOffset);
    }

    return obstacleDetected;
}

bool NavigateCommand::CheckEndConditionNoEndWall(int angleStart, int angleEnd)
{
    int startIndex = AngleToIndex(angleStart);
    int endIndex = AngleToIndex(angleEnd);
    int nbAngles = 0;

    for (int i = startIndex; i != endIndex; i = (i + 1) % 360)
    {
        if (m_drive->lidar.GetData().distance[i] >= 20 && m_drive->lidar.GetData().distance[i] < endNoObstacleDistance)
        {
            nbAngles++;
            if (nbAngles >= endNbAnglesTolerance)
                return false;
        }
    }

    return true;
}

bool NavigateCommand::CheckEndCondition()
{
    float robotYaw = NormalizeAngle(m_drive->GetYaw());

    if ((robotYaw > 360 - endRotTolerance) || robotYaw < endRotTolerance) 
    {
        int angleFrontLeftRelative = static_cast<int>(NormalizeAngle(angleFrontLeft - robotYaw - lidarOffset));
        int angleFrontRightRelative = static_cast<int>(NormalizeAngle(angleFrontRight - robotYaw - lidarOffset));
        int angleFrontRelative = static_cast<int>(NormalizeAngle(angleFront - robotYaw - lidarOffset));

        bool isFrontEnd = GetAverageDistanceInZone(AdjustAngleForLidarOffset(angleFront - endRotTolerance), AdjustAngleForLidarOffset(angleFront + endRotTolerance)) < endFrontSideThreshold;
        float angleLeftDist = GetAverageDistanceInZone(AdjustAngleForLidarOffset(angleFrontLeft), AdjustAngleForLidarOffset(angleLeft));
        float angleRightDist = GetAverageDistanceInZone(AdjustAngleForLidarOffset(angleRight), AdjustAngleForLidarOffset(angleFrontRight));
        bool areSidesEnd = (angleLeftDist < endCloseSideThreshold && angleRightDist < endFarSideThreshold && angleRightDist > endMinFarSideThreshold) ||
                           (angleRightDist < endCloseSideThreshold && angleLeftDist < endFarSideThreshold && angleLeftDist > endMinFarSideThreshold);

        end = true;
        SendControl(0, 0);
        return areSidesEnd && isFrontEnd;
    }
    return false;
}

float NavigateCommand::DetermineTurnDirection()
{
    float averageLeft, averageRight = 0;

    float robotYaw = NormalizeAngle(m_drive->GetYaw());

    int angleFrontLeftRelative = NormalizeAngle(angleFrontLeft - robotYaw - lidarOffset);
    int angleFrontRightRelative = NormalizeAngle(angleFrontRight - robotYaw - lidarOffset);
    int obstacleYaw = NormalizeAngle(obstacleId + lidarOffset + robotYaw);

    //hell of if else
    if (robotYaw >= angleFrontRight && robotYaw <= 180) // Check if robot isn't in the wrong way from the left
    {
        averageRight = -1;
    }
    else if (robotYaw <= angleFrontLeft && robotYaw >= 180) // Check if robot isn't in the wrong way from the right
    {
        averageLeft = -1;
    }
    else if (obstacleYaw > angleFrontLeft || obstacleYaw < angleFrontRight) // Then, check if the obstacle is in front of the robot
    {
        averageLeft = CalculateMedianDistance(angleFrontLeftRelative, obstacleId);
        averageRight = CalculateMedianDistance(obstacleId, angleFrontRightRelative);
    }
    else if (obstacleYaw >= angleFrontRight && obstacleYaw <= 180) // Otherwhise, check if obstacle is on the left
    {
        averageRight = -1;
    }
    else if (obstacleYaw <= angleFrontLeft && obstacleYaw >= 180) // Otherwhise, check if obstacle is on the right
    {
        averageLeft = -1;
    }
    // else, will recompute on the next iteration.

    return averageLeft < averageRight ? 1.0 : -1.0; // turn left, or right
}

void NavigateCommand::TurnInDirection(float direction)
{
    float angularVelocity = direction * maxAngularVelocity;
    logFile << "TURN" << std::endl;
    SendControl(0.0, angularVelocity);
}

int NavigateCommand::AdjustAngleForLidarOffset(int angle)
{
    return NormalizeAngle(angle + lidarOffset);
}

bool NavigateCommand::IsAngleInRange(int angle, int startAngle, int endAngle)
{
    if (startAngle < endAngle)
    {
        return angle >= startAngle && angle <= endAngle;
    }
    else
    {
        return angle >= startAngle || angle <= endAngle;
    }
}

/**************************Algo 2************************************/
void NavigateCommand::WaypointsInit()
{
    // Waypoint : TargetYaw (turning final angle before moving), WallThreshold (mm from wall to stop the robot), AngleToCheck (at which angle it checks the wall)
    waypoints = {
        {0, 450.0f, 0},   // Waypoint 1 : TargetYaw, WallThreshold, AngleToCheck
        {180, 450.0f, 0},  
        {0, 450.0f, 0}  
    };
    endWaypoint = {180, 400.0f, 0}; 
}

void NavigateCommand::FollowWall()
{
    if (currentStep >= static_cast<int>(waypoints.size()))
    {
        end = true;
        SendControl(0, 0);
        return;
    }

    Waypoint currentWaypoint = waypoints[currentStep];

    logFile.open("/home/pi/nav.log", std::ios_base::trunc);
    logFile << "state ID: " << (int)currentState << std::endl;
    logFile.close();

    switch (currentState)
    {
        case FollowWallState::None:
            currentState = FollowWallState::MovingForward;
            break;

        case FollowWallState::MovingForward:
        {
            Forward();
            float av = GetAverageDistanceInZone(NormalizeAngle(currentWaypoint.AngleToCheck - 5), NormalizeAngle(currentWaypoint.AngleToCheck + 5));
            if (av < currentWaypoint.WallThreshold)
            {
                currentStep++;
                if (currentStep < static_cast<int>(waypoints.size()))
                {
                    currentState = FollowWallState::Turning;
                    Turn(waypoints[currentStep].TargetYaw);
                }
                else
                {
                    currentState = FollowWallState::End;
                }
            }

            break;
        }

        case FollowWallState::Turning:
            Turn( currentWaypoint.TargetYaw);
            if (fabs(robotYaw - currentWaypoint.TargetYaw) < 5)
            {
                SendControl(0, 0);
                currentState = currentStep >= static_cast<int>(waypoints.size()) ? FollowWallState::End : FollowWallState::None;
            }
            break;

        case FollowWallState::End:
        {
            Forward();
            float bv = GetAverageDistanceInZone(NormalizeAngle(endWaypoint.AngleToCheck - 5), NormalizeAngle(endWaypoint.AngleToCheck + 5));
            if (bv > endWaypoint.WallThreshold)
            {
                SendControl(0, 0);
                currentState = FollowWallState::Stop;
                end = true;
            }
            break;
        }

        case FollowWallState::Stop:
            SendControl(0, 0);
            break;
    }
}

void NavigateCommand::Forward()
{
    float currentYaw = m_drive->GetYaw();
    float angleDifference = NormalizeAngle(currentYaw - waypoints[currentStep].TargetYaw);
    float direction = angleDifference > 0 ? -1.0f : 1.0f;
    float angularVelocityCorrection = std::clamp(maxAngularVelocity * direction * fabs(angleDifference), -maxAngularVelocity, maxAngularVelocity);

    SendControl(maxLinearVelocity, angularVelocityCorrection);
}

void NavigateCommand::Turn(int targetYaw)
{
    float currentYaw = m_drive->GetYaw();
    float angleDifference = NormalizeAngle(currentYaw - targetYaw);
    float direction = angleDifference > 0 ? -1.0f : 1.0f;
    float angularVelocity = maxAngularVelocity * direction;

    SendControl(0, angularVelocity);
}

/**************************Algo 3************************************/
void NavigateCommand::WallFollowing()
{
    float leftWallDistance = GetAverageDistanceInZone(NormalizeLidarAngle(leftWallAngle - lidarAngleRange), NormalizeLidarAngle(leftWallAngle + lidarAngleRange));
    float rightWallDistance = GetAverageDistanceInZone(NormalizeLidarAngle(rightWallAngle - lidarAngleRange), NormalizeLidarAngle(rightWallAngle + lidarAngleRange));

    float frontLeftWallDistance = GetAverageDistanceInZone(NormalizeLidarAngle(frontLeftAngle - lidarAngleRange), NormalizeLidarAngle(frontLeftAngle + lidarAngleRange));
    float frontRightWallDistance = GetAverageDistanceInZone(NormalizeLidarAngle(frontRightAngle - lidarAngleRange), NormalizeLidarAngle(frontRightAngle + lidarAngleRange));
    float frontWallDistance = GetAverageDistanceInZone(NormalizeLidarAngle(-lidarAngleRange), NormalizeLidarAngle(lidarAngleRange));
    
    float distanceAhead = 0.0f;
    float distanceToWallAhead = 0.0f;

    logFile.open("/home/pi/nav.log", std::ios_base::trunc);
    logFile << "nav state " << (int)currentWallFollowingState << std::endl;


    // if (!isInitialMovement && frontLeftWallDistance > wallEndThreshold && frontRightWallDistance > wallEndThreshold && frontWallDistance > wallEndThreshold && leftWallDistance > wallEndThreshold && rightWallDistance > wallEndThreshold)
    // {
    //     SendControl(0, 0);
    //     currentWallFollowingState = WallFollowingState::Finished;
    //     return;
    // }

    switch (currentWallFollowingState)
    {
        case WallFollowingState::Idle:
            currentWallFollowingState = WallFollowingState::PrecisionMovement;
            initialFrontWallDistance = GetAverageDistanceInZone(NormalizeLidarAngle(-lidarAngleRange), NormalizeLidarAngle(lidarAngleRange));
            targetMovementDistance = startMovementDistance;
            targetRotationYaw = 270; // OR 90 for RIGHT
            break;

        case WallFollowingState::PrecisionMovement:
            MoveForwardWithoutCorrection();
            distanceAhead = GetAverageDistanceInZone(NormalizeLidarAngle(-lidarAngleRange), NormalizeLidarAngle(lidarAngleRange));
                logFile << "initialFrontWallDistance : " << initialFrontWallDistance  << " distanceAhead : " << distanceAhead << std::endl;
            if (fabs(initialFrontWallDistance - distanceAhead) > targetMovementDistance || distanceAhead <= minimumFrontWallDistance)
            {
                currentWallFollowingState = WallFollowingState::Turning;
                RotateToYaw(targetRotationYaw);
            }
            break;

        case WallFollowingState::MovingToWall:
            MoveForwardWithCorrection();
            distanceToWallAhead = GetAverageDistanceInZone(NormalizeLidarAngle(-lidarAngleRange), NormalizeLidarAngle(lidarAngleRange));
            if (distanceToWallAhead < minimumFrontWallDistance)
            {
                SendControl(0, 0);
                currentWallFollowingState = WallFollowingState::FollowingWall;
            }
            break;

        case WallFollowingState::Turning:
                logFile << "yaw : " << robotYaw << " target : " << targetRotationYaw << std::endl;
            SendControl(0, lastAngularVel);
            if (fabs(robotYaw- targetRotationYaw) < 5)
            {
                SendControl(0, 0);
                if (isInitialMovement)
                {
                    currentWallFollowingState = WallFollowingState::MovingToWall;
                    isInitialMovement = false;
                }
                else if (hasLostLeftWall)
                {
                    targetMovementDistance = distanceToCrossLostWall;
                    initialFrontWallDistance = GetAverageDistanceInZone(NormalizeLidarAngle(-lidarAngleRange), NormalizeLidarAngle(lidarAngleRange));
                    targetRotationYaw = NormalizeLidarAngle(targetRotationYaw - 90);
                    hasLostLeftWall = false;
                    currentWallFollowingState = WallFollowingState::PrecisionMovement;
                }
                else if (hasLostLeftWallAgain)
                {
                    targetMovementDistance = distanceToCrossLostWall;
                    initialFrontWallDistance = GetAverageDistanceInZone(NormalizeLidarAngle(-lidarAngleRange), NormalizeLidarAngle(lidarAngleRange));
                    hasLostLeftWallAgain = false;
                    currentWallFollowingState = WallFollowingState::PrecisionMovement;
                }
                else
                {
                    currentWallFollowingState = WallFollowingState::FollowingWall;
                }
            }
            break;

        case WallFollowingState::FollowingWall:
            if (leftWallDistance < sideWallDistance) // Following the left wall
            {
                if (frontWallDistance > minimumFrontWallDistance && frontLeftWallDistance > minimumFrontWallDistance && frontRightWallDistance > minimumFrontWallDistance)
                {
                    MoveForwardWithCorrection();
                }
                else
                {
                    float sideWallDistance = GetAverageDistanceInZone(40 - lidarAngleRange, 40 + lidarAngleRange);
                    int wallAngle = CalculateWallAngle(frontWallDistance, sideWallDistance, 40);
                    targetRotationYaw = NormalizeLidarAngle(static_cast<int>(round(robotYaw/ 5.0f) * 5) + (90 - (90 - wallAngle)));
                    currentWallFollowingState = WallFollowingState::Turning;
                    RotateToYaw(targetRotationYaw);
                }
            }
            else
            {
                targetMovementDistance = distanceToMoveWhenWallLost;
                initialFrontWallDistance = GetAverageDistanceInZone(NormalizeLidarAngle(-lidarAngleRange), NormalizeLidarAngle(lidarAngleRange));
                targetRotationYaw = NormalizeLidarAngle(targetRotationYaw - 90);
                hasLostLeftWall = true;
                hasLostLeftWallAgain = true;
                currentWallFollowingState = WallFollowingState::PrecisionMovement;
            }
            break;
        default:
            break;
    }

        logFile.close();
}

int NavigateCommand::CalculateWallAngle(float frontDistance, float sideDistance, float angleBetween)
{
    float angleInRadians = angleBetween * M_PI / 180.0f;
    float hypotenuse = sqrt(frontDistance * frontDistance + sideDistance * sideDistance - 2 * frontDistance * sideDistance * cos(angleInRadians));

    float sineAngle = sideDistance * sin(angleInRadians) / hypotenuse;
    float angleToWall = asin(sineAngle) * 180.0f / M_PI;

    return static_cast<int>(round(angleToWall / 5.0f) * 5);
}

void NavigateCommand::MoveForwardWithoutCorrection()
{
    SendControl(maxLinearVelocity, 0);
}

void NavigateCommand::MoveForwardWithCorrection()
{
    SendControl(maxLinearVelocity, 0);
    return;
    float currentYaw = m_drive->GetYaw();
    float angleDifference = NormalizeLidarAngle(currentYaw - targetRotationYaw);
    float direction = angleDifference > 0 ? -1.0f : 1.0f;
    float angularVelocityCorrection = std::clamp(maxAngularVelocity * direction * fabs(angleDifference), -maxAngularVelocity, maxAngularVelocity);

    SendControl(maxLinearVelocity, angularVelocityCorrection);
}

void NavigateCommand::RotateToYaw(int targetYaw)
{
    float currentYaw = m_drive->GetYaw();
    float angleDifference = NormalizeLidarAngle(currentYaw - targetYaw);
    float direction = angleDifference > 0 ? -1.0f : 1.0f;
    float angularVelocity = maxAngularVelocity * direction;
    lastAngularVel = angularVelocity;
    SendControl(0, angularVelocity);
}

int NavigateCommand::ConvertAngleToLidarIndex(int angle)
{
    int index = (angle - lidarOffset + 360) % 360;
    return index;
}

int NavigateCommand::NormalizeLidarAngle(int angle)
{
    int index = (angle + 360) % 360;
    return index;
}

/**************************Algo 4************************************/
void NavigateCommand::Following()
{
    if(finish)
        return; 

    static auto startTime = std::chrono::steady_clock::now();
    
    // enter board
    if (starting)
    {
        SendControl(maxLinearVelocity, 0.0f); 

        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsedTime = currentTime - startTime;
        if (elapsedTime.count() >= 2.0f)
        {
            starting = false;
        }
        return;
    }

    logFile << "Starting Following method" << std::endl;


    //step one
    m_drive->scanData = m_drive->lidar.GetData();
    std::vector<float> zoneDistances(total_zones, 0.0f);
    for (int zone = 0; zone < total_zones; ++zone)
    {
        float sum = 0;
        int startAngle = NormalizeAngle(lidar_start_angle + zone * lidar_angle_range);
        int endAngle = NormalizeAngle(lidar_start_angle + (zone + 1) * lidar_angle_range);
        zoneDistances[zone] = CalculateMedianDistance(startAngle, endAngle);

        //check if no close wall at specified angles
        if(startAngle >= front_left_angle && endAngle >= front_left_angle && endAngle <= front_left_angle + lidar_angle_range)
        {
            front_left_distance = zoneDistances[zone];
        } else if (startAngle >= front_right_angle && endAngle >= front_right_angle && endAngle <= front_right_angle + lidar_angle_range)
        {
            front_right_distance = zoneDistances[zone];
        }
        logFile << "Zone " << startAngle <<  " - " << endAngle << " - Med Distance: " << zoneDistances[zone] << std::endl;  
        
    }
        logFile << "front_right_distance " << front_right_distance <<  " | front_left_distance " << front_left_distance << std::endl;  

    //step 2
    float sum = 0;
    for (int i = 0; i < zoneDistances.size(); i++)
    {
        sum += zoneDistances[i];
    }
    float av = (sum/zoneDistances.size());
    logFile <<" - Av  Distances: " << av << std::endl;

    //check end
    if (av < cul_de_sac_threshold)
    {
        logFile << "END too close, stopping" << std::endl;
        m_drive->SetRunningLED(false);
        SendControl(0.0f, 0.0f); 
        finish = true;
        return;
    }

    //step 4
    float angularVelocity = CalculateAngularVelocity(zoneDistances); 
    SendControl(maxLinearVelocity, angularVelocity);
    
}

   const int TOTAL_GROUPS = 4;

float NavigateCommand::CalculateAngularVelocity(const std::vector<float>& zoneDistances)
{
    //adapt robot according wall, priority over following lidar
    const float SAFE_DISTANCE = 250.0f;  
    float diff = 0;
    if (front_left_distance <= close_threshold && front_left_distance <= front_right_distance)
    {
        diff = close_threshold - front_left_distance;
        float angular_velocity = (diff / (close_threshold - SAFE_DISTANCE)) * maxAngularVelocity;
        angular_velocity = std::clamp(angular_velocity, 0.0f, maxAngularVelocity);

        logFile << "Near Left Wall - Adjusting Angular Velocity: " << angular_velocity << std::endl;
        return angular_velocity; 
    }
    else if (front_right_distance <= close_threshold && front_right_distance <= front_left_distance)
    {
        diff = close_threshold - front_right_distance;
        float angular_velocity = -(diff / (close_threshold - SAFE_DISTANCE)) * maxAngularVelocity;
        angular_velocity = std::clamp(angular_velocity, -maxAngularVelocity, 0.0f);

        logFile << "Near Right Wall - Adjusting Angular Velocity: " << angular_velocity << std::endl;
        return angular_velocity;
    }

    //
    float groupAverages[total_groups] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int group = 0; group < total_groups; ++group)
    {
        float sum = 0.0f;
        for (int i = 0; i < group_size; ++i)
        {
            sum += zoneDistances[group * group_size + i];
        }
        groupAverages[group] = sum / group_size;
    }

    float max_group_distance = *std::max_element(groupAverages, groupAverages + total_groups);
    int max_group_index = std::distance(groupAverages, std::max_element(groupAverages, groupAverages + total_groups));

    int group_start_angle = lidar_start_angle + max_group_index * (group_size * lidar_angle_range);
    int group_end_angle = group_start_angle + (group_size * lidar_angle_range);
    
    float max_distance = -1.0f;
    int target_zone_index = -1;
    
    for (int i = 0; i < group_size; ++i)
    {
        int zone_index = max_group_index * group_size + i;
        if (zoneDistances[zone_index] > max_distance)
        {
            max_distance = zoneDistances[zone_index];
            target_zone_index = zone_index;
        }
    }
    
    float target_angle = NormalizeAngle(lidar_start_angle + target_zone_index * lidar_angle_range + lidar_angle_range / 2);

    float direction = 1.0f;
    int rel_angle = target_angle;
    if(rel_angle > 90.0f)
    {
        rel_angle = 360 - rel_angle;
        direction = -1;
    }

    float angular_velocity = (rel_angle / 40.0f) * maxAngularVelocity * direction;

    if (rel_angle > 10) {
        angular_velocity *= 2.0f; 
    } else {
        angular_velocity *= 1.0f; 
    }

    logFile << "angular_velocity bef: " << angular_velocity << std::endl;

    angular_velocity = std::clamp(angular_velocity, -maxAngularVelocity, maxAngularVelocity);
        logFile << "Selected Group Index: " << max_group_index
            << " | Target Zone Index: " << target_zone_index
            << " | Target Angle: " << target_angle
            << " | Relative Angle: " << rel_angle
            << " | Angular Velocity: " << angular_velocity << std::endl;


    return angular_velocity;
}
