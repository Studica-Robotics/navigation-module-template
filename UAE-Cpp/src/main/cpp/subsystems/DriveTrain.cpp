#include "subsystems/DriveTrain.h"

#define DEBUG true

DriveTrain::DriveTrain()
{
    ResetYaw();

    //Motor Invert flags
    frontLeftMotor.SetInverted(false);
    backLeftMotor.SetInverted(false);
    frontRightMotor.SetInverted(false);
    backRightMotor.SetInverted(false);    lidar.ClusterConfig(50.0f, 5);
    
    lidar.EnableFilter(studica::Lidar::Filter::kCLUSTER, true);

}

double DriveTrain::GetYaw()
{
    return navX.GetYaw();
}

double DriveTrain::GetAngle()
{
    return navX.GetAngle() *-1;
}

void DriveTrain::ResetYaw()
{
    navX.ZeroYaw();
}

bool DriveTrain::GetStartButton()
{
    return startButton.Get();
}

bool DriveTrain::GetEStopButton()
{
    return eStopButton.Get();
}

void DriveTrain::SetRunningLED(bool on)
{
    runningLED.Set(on);
}

void DriveTrain::SetStoppedLED(bool on)
{
    stoppedLED.Set(on);
}

void DriveTrain::LidarStop()
{
    if (!lidarRunning)
    {
        lidar.Start();
        lidarRunning = true;
    }
}

void DriveTrain::LidarStart()
{
    if (lidarRunning)
    {
        lidar.Stop();
        lidarRunning = false;
    }
}

    void DriveTrain::StackMotorControl(double x, double y)
    {
        frc::SmartDashboard::PutNumber("x",x);
        frc::SmartDashboard::PutNumber("y",y);
        frontLeftMotor.Set(y + x);
        backLeftMotor.Set(y + x);
        frontRightMotor.Set(y - x);
        backRightMotor.Set(y - x);
        
    }

void DriveTrain::TwoWheelMotorControl(double x, double y)
{
    frontLeftMotor.Set(y + x);
    frontRightMotor.Set(y - x);
}

void DriveTrain::SixWheelMotorControl(double x, double y)
{
    frontLeftMotor.Set(y + x);
    backLeftMotor.Set(y + x);
    frontRightMotor.Set(y - x);
    backRightMotor.Set(y - x);
}

void DriveTrain::MecanumMotorControl(double x, double y, double z)
{
    denomonator = fmax(fabs(y) + fabs(x) + fabs(z), 1.0);
    frontLeftMotor.Set(y + (x) + z / denomonator);
    backLeftMotor.Set(y - (x) + z / denomonator);
    frontRightMotor.Set(y - (x) - z / denomonator);
    backRightMotor.Set(y + (x) - z / denomonator);
}

void DriveTrain::XBotMotorControl(double x, double y, double z)
{
    denomonator = fmax(fabs(y) + fabs(x) + fabs(z), 1.0);
    frontLeftMotor.Set(y + (x) + z / denomonator);
    backLeftMotor.Set(y - (x) + z / denomonator);
    frontRightMotor.Set(y - (x) - z / denomonator);
    backRightMotor.Set(y + (x) - z / denomonator);
}

void DriveTrain::HolonomicDrive(double x, double y, double z)
{
    denomonator = fmax(fabs(y) + fabs(x) + fabs(z), 1.0);
    frontLeftMotor.Set((((x / 3) + (y / sqrt(3)) + z) * sqrt(3))/denomonator);
    backLeftMotor.Set((((x / 3) - (y / sqrt(3)) + z) * sqrt(3))/denomonator);
    frontRightMotor.Set((((-2 * x / 3) + z) * sqrt(3))/denomonator);
}

void DriveTrain::Periodic()
{

}


double DriveTrain::MedianFilter(const std::vector<double>& data, size_t windowSize) {
    std::vector<double> window(data.begin(), data.end());
    std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
    return window[window.size() / 2];
}

void DriveTrain::ProcessLidarData(const std::vector<double>& raw_data, std::vector<double>& processed_data) {
    if (!raw_data.empty()) {
        size_t windowSize = 5;
        for (size_t i = 0; i < raw_data.size(); i++) {
            std::vector<double> window;
            for (size_t j = std::max(int(i) - int(windowSize) / 2, 0);
                 j <= std::min(i + windowSize / 2, raw_data.size() - 1);
                 j++) {
                window.push_back(raw_data[j]);
            }
            processed_data.push_back(MedianFilter(window, window.size()));
        }
    }
}

void DriveTrain::UpdateLidarReadings() {
    std::vector<double> raw_front_readings, raw_left_readings,raw_right_readings;

    for (size_t i = 65; i <=105 ; i++) {
        double dist =  GetLidarDist(i);
        if (dist >= 0.1) front_lidar_readings.push_back(dist);
    }
    for (size_t i = 0; i <= 15; i++) {
        double dist =  GetLidarDist(i);
         left_lidar_readings.push_back(dist);
    }
}


double DriveTrain::CalculateMean(const std::vector<double> &data) {
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    return sum / data.size();
}

double DriveTrain::CalculateStdDev(const std::vector<double>& data, double mean) {
    std::vector<double> diff(data.size());
    std::transform(data.begin(), data.end(), diff.begin(), [mean](double x) { return x - mean;});
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    return std::sqrt(sq_sum / data.size());
}

bool DriveTrain::DetectObstacle(double min_dist, double mean, double stddev) {
    double threshold = 0.6;
    return normalCDF(min_dist, mean, std::sqrt(stddev)) >= threshold;
}


double DriveTrain::normalCDF(double value, double mean, double stddev) {
    return 0.5 * (1 + std::erf((value - mean) / (stddev * std::sqrt(2))));
}


void DriveTrain::UpdateSensorReadings() {

    front_mean_dist = CalculateMean(front_lidar_readings);
    front_stddev = CalculateStdDev(front_lidar_readings, front_mean_dist);
    front_obj_flag = DetectObstacle(0.25, front_mean_dist, front_stddev);

    left_mean_dist = CalculateMean(left_lidar_readings);
    left_stddev = CalculateStdDev(left_lidar_readings, left_mean_dist);
    left_obj_flag = DetectObstacle(0.55, left_mean_dist, left_stddev);

    right_mean_dist = CalculateMean(right_lidar_readings);
    right_stddev    = CalculateStdDev(right_lidar_readings, right_mean_dist);
    right_obj_flag  = DetectObstacle(0.55, right_mean_dist, right_stddev);

    front_lidar_readings.clear();
    left_lidar_readings.clear();
    right_lidar_readings.clear();

}


std::pair<double, double> DriveTrain::ArcCompute(double angular_vel, double rotation_radius) {
    double linear_velocity = angular_vel * rotation_radius;
    return std::make_pair(linear_velocity, angular_vel);
}



void DriveTrain::UpdateReadings() {
    UpdateLidarData();
    UpdateLidarReadings();
    UpdateSensorReadings();
}

double DriveTrain::GetGlobalAngle()
{
    double angle = remainder(GetAngle(),360);
    if(angle<0)angle+=360;
    return angle;
}

double DriveTrain::GetLidarDist(int degree)
{
    return (scanData.distance[degree] / 1000);
}

void DriveTrain::UpdateLidarData()
{
    scanData = lidar.GetData();
}

// to identify the points of the lidar
std::pair<double, double> DriveTrain::PolarToCartesian(int angleDegrees)
{
    double distance = GetLidarDist(angleDegrees);
    double angleRadians = angleDegrees * (M_PI / 180.0);  // Convert degrees to radians
    double x = distance * std::cos(angleRadians);
    double y = distance * std::sin(angleRadians);
    return std::make_pair(x, y);
}

void DriveTrain::StopMotor()
{
    frc::Wait(0.004);
    frontLeftMotor.StopMotor();
    frc::Wait(0.004);
    backLeftMotor.StopMotor();
    frc::Wait(0.004);
    frontRightMotor.StopMotor();
    frc::Wait(0.004);
    backRightMotor.StopMotor();
}