#include <commands/Navigation.h>

// Aligned Angle is 85


#define TimeToEnterCurt 2
#define SpeedToEnterCurt 0.3
#define AngularVelocityLeft1 0.3
#define AngularVelocityLeft2 0.33
#define AngularVelocityRight 0.3
#define LidarPoint1 0

Navigation::Navigation(DriveTrain* drive):drive{drive}
{
AddRequirements({drive});
}
void Navigation::Initialize() 
{
    drive->ResetYaw();
    IsRobotState = RobotState::FolloWall;
    frc::SmartDashboard::PutBoolean("test",0);
    robot_in_curt = true;

}
void Navigation::Execute() 
{
    

    frc::SmartDashboard::PutNumber("Angle",CheckLidarAngle(0,10));
    if(is_current_state_done&& !robot_in_curt)
    {
         IsRobotState = RobotState::CuretEnturenc;
    }
    else {
        if((drive->left_obj_flag && !drive->front_obj_flag)  && is_current_state_done)
        {
            IsRobotState = RobotState::FolloWall;
        }
        else if(((drive->left_obj_flag && drive->front_obj_flag) || (!drive->left_obj_flag && drive->front_obj_flag))&& is_current_state_done)
        {
            isRotateRight = true;
            isRotateLeft = false;
            IsRobotState = RobotState::RotateRight;
        }
        else if(!drive->left_obj_flag && !drive->front_obj_flag &&is_current_state_done)
        {
            isRotateRight = false;
            isRotateLeft = true;
            IsRobotState = RobotState::RotatLeft;
        }
    }

    switch (IsRobotState)
    {
    case RobotState::CuretEnturenc: HandleCurtEnturence(); break;
    case RobotState::Idel: HandleIdleState(); break;
    case RobotState::FolloWall: HandleFollowingWallState(); break;
    case RobotState::RotateRight: HandleRotateRightState(); break;
    case RobotState::RotatLeft: HandleRotateLeftState(); break;

    default:
        break;
    }
    // drive->StackMotorControl(0,0.2);

}
void Navigation::End(bool interrupted) 
{

}
bool Navigation::IsFinished() 
{
    return false;
}


void Navigation::HandleIdleState()
{
    is_current_state_done = false;
    if(do_once_flag)
    {
        ResetFlags();
        do_once_flag = false;
        last_time =  std::chrono::high_resolution_clock::now();
        drive->StopMotor();
    }

    current_time =  std::chrono::high_resolution_clock::now();

    delta_time = current_time - last_time;
    if(delta_time.count() >=0.1)
    {
        drive->StopMotor();
        is_current_state_done = true;
        do_once_flag = true;
    }
}

void Navigation::HandleCurtEnturence()
{
    is_current_state_done = false;
    if(do_once_flag )
    {
        do_once_flag = false;
        last_time =  std::chrono::high_resolution_clock::now();

    }

    current_time =  std::chrono::high_resolution_clock::now();

    delta_time = current_time - last_time;
    if(delta_time.count() < TimeToEnterCurt)
    {
            drive->StackMotorControl(SpeedToEnterCurt,0);
    }
    else
    {
        do_once_flag = true;
        robot_in_curt = true;
        IsRobotState = RobotState::Idel;
    }
    
}

void Navigation::HandleFollowingWallState()
{
    is_current_state_done = false;
    if (do_once_flag)
    {
        do_once_flag = false;
        last_time =  std::chrono::high_resolution_clock::now();
    }
    current_time =  std::chrono::high_resolution_clock::now();
    delta_time = current_time - last_time;

    if( drive->front_obj_flag || delta_time.count()>2)
    {
        do_once_flag = true;
        IsRobotState = RobotState::Idel;
    }
    else
    {
        auto velocity = CalculateVelocity();
        frc::SmartDashboard::PutNumber("angular",velocity.second);   
        drive->MecanumMotorControl(0,velocity.first,velocity.second);
    }
}

void Navigation::HandleRotateLeftState()
{
    HandleRotationStation(true);
}

void Navigation::HandleRotateRightState()
{
    HandleRotationStation(false);
}
void Navigation::HandleRotationStation(bool is_rotate_left)
{
    is_current_state_done = false;
    currentAngle = drive->GetGlobalAngle();
    if(do_once_flag)
    {
        do_once_flag = false;
        targetAngle = int(currentAngle + (is_rotate_left? 45:-45) +360)%360;
        if (is_rotate_left)
        {
            rotation_counter++;
        }
        else
        {
            rotation_counter = 0;
        }
        
    }

    if(std::abs(currentAngle-targetAngle)< 2)
    {
        isRotateLeft = false;
        isRotateRight = false;
        do_once_flag = true;
        IsRobotState = RobotState::Idel;
    }
    else
    {
        auto angular_compute = drive->ArcCompute(rotation_counter == 1 ? AngularVelocityLeft1 :AngularVelocityLeft2 , rotation_counter == 1 ? 1.1:1.1);
        drive->StackMotorControl(is_rotate_left ?angular_compute.first:0,is_rotate_left?-angular_compute.second:AngularVelocityRight);
        frc::SmartDashboard::PutNumber("angular_compute.first",angular_compute.first);
    }
    

}
void Navigation::ResetFlags()
{
    isRotateRight = false;
    isRotateLeft = false;    
}
std::pair<double,double> Navigation::CalculateVelocity()
{
    
    const double target_distance = 0.35; // Target distance to the wall in meters
    const double dist_tolorance = 0.04;
    const int target_angle = 85;
    double angle_tolerance = 10;

    


    double angle_diff_degree = CheckLidarAngle(LidarPoint1,LidarPoint1+10);

    double angular_speed = 0;
    double linear_speed = 0;

     if (((abs(angle_diff_degree-target_angle)<= angle_tolerance) && (abs(drive->left_mean_dist- target_distance) <dist_tolorance ))) 
    {
        double angle_speed_angle = std::clamp(0.016 * angle_diff_degree -1.36 ,-0.1,0.1); 
        angular_speed = angle_speed_angle;
        linear_speed  = std::clamp((-0.0011*angle_diff_degree*angle_diff_degree) + (0.1943*angle_diff_degree)- 7.9,0.1,0.4);
    } 
    else {
        double angle_speed_angle = std::clamp(0.016 * angle_diff_degree -1.36 ,-0.15,0.15); 
        double angle_speed_wall = std::clamp(-1.4* drive->left_mean_dist + 0.49,-0.1,0.1);

        angular_speed = std::clamp(angle_speed_angle+angle_speed_wall,-0.15,0.15);
        // angular_speed = std::max( std::min( angular_speed, 0.15 ), -0.15 );
        frc::SmartDashboard::PutNumber("angular_speed",angular_speed);
        linear_speed  = std::clamp((-0.0017*angle_diff_degree*angle_diff_degree) + (0.2914*angle_diff_degree)- 12.08,0.15,0.4);
    }

    return {linear_speed,angular_speed};
}


int Navigation::CheckLidarAngle(int angle1,int angle2)
{
    double lidar_angle_diff = angle1 - angle2;
    double a = drive->GetLidarDist(angle1);  // in meters
    double b = drive->GetLidarDist(angle2);  // in meters
    double C_degrees = abs(lidar_angle_diff);     
    double C_radians = C_degrees * M_PI / 180.0;
    double c = std::sqrt((a*a + b*b) - (2*a*b * std::cos(C_radians)));
    double A_radians = std::acos((b*b + c*c - a*a)/(2*c*b));
    double A_degree = (A_radians * (180 / M_PI)) ;
    return (A_degree);
}
