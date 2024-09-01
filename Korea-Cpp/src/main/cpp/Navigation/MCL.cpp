#include <chrono>
#include <opencv2/imgproc.hpp>

#include "Navigation/MCL.h"
#include "Constants.h"

MCL::MCL(unsigned char **map) : map_(map)
{
    generator_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());

    initialized_ = false;
    mcl_pose_[0] = mcl_pose_[1] = mcl_pose_[2] = 0;

    prev_time_ = 0;
    particles_ = new Particle[particle_max_];
}
MCL::~MCL()
{
    delete[] particles_;
}
void MCL::Init()
{
    Particle particle_tmp;

    for (int i = 0; i < particle_max_;)
    {
        particle_tmp.pose[0] = 0;
        particle_tmp.pose[1] = 0;

        Vector2i grid_pose = ConvertToGridCoordinate({particle_tmp.pose[0], particle_tmp.pose[1], 0});
        if (!IsSafe(grid_pose, Constants::MAP_SIZE) || map_[grid_pose[0]][grid_pose[1]] < 128)
            continue;

        particle_tmp.pose[2] = Interface::GyroYaw;
        particle_tmp.weight = 1.0 / particle_max_;

        particles_[i] = particle_tmp;
        i++;
    }

    initialized_ = true;
    prev_time_ = 0;
    mcl_pose_[0] = mcl_pose_[1] = mcl_pose_[2] = 0;
}
void MCL::Reset()
{
    Interface::RobotPose[0] = Interface::RobotPose[1] = Interface::RobotPose[2] = 0;

    Particle p;
    p.pose[0] = p.pose[1] = p.pose[2] = 0;
    for (int i = 0; i < particle_max_; i++) particles_[i] = p;
    
    initialized_ = false;
}
void MCL::Update()
{
    if (!initialized_)
    {
        Init();
        return;
    }

    Prediction();
    ComputeParticlesWeight();
    Resampling();
    OdometryMCLFusion();
}
void MCL::Prediction()
{
    double xy_noise_gain, th_noise_gain, predict_vel = Interface::Velocity[0];

    if(DRIVE_TYPE == Mecanum_Drive)
    {
        predict_vel *= 0.65;
        if(std::abs(Interface::Velocity[0]) < 190) xy_noise_gain = 0;
        else xy_noise_gain = Trans(190, 500, 0, 10, std::abs(predict_vel));
    }
    else
    {
        if(DRIVE_TYPE == SixWheel_Drive) predict_vel *= 0.9;
        xy_noise_gain = Trans(0, 500, 0, 10, std::abs(predict_vel));
    }
    th_noise_gain = Trans(0, 140, 0.5, 6, std::abs(Interface::Velocity[2]));

    std::uniform_real_distribution<double> xy_noise(-xy_noise_gain * 0.5, xy_noise_gain);
    std::uniform_real_distribution<double> th_noise(-ToRadian(th_noise_gain), ToRadian(th_noise_gain));

    double r = 0;
    int current_time = GetTime();
    if(prev_time_) r = (predict_vel * (current_time - prev_time_)) / 1000.0;
    prev_time_ = current_time;

    for (int i = 0; i < particle_max_; i++)
    {
        particles_[i].pose[2] = NormalizeRadian(Interface::GyroYaw + th_noise(generator_));
        particles_[i].pose[0] += r * std::cos(particles_[i].pose[2]) + (xy_noise(generator_) * std::cos(particles_[i].pose[2])) * sign(r);
        particles_[i].pose[1] += r * std::sin(particles_[i].pose[2]) + (xy_noise(generator_) * std::sin(particles_[i].pose[2])) * sign(r);
    }
}
double MCL::ComputeWeightFromPose(const Vector3d &pose)
{
    double r, th, weight = 0;
    int availableDataCnt = 0;
    
    for (int i = 0; i < 450; i++)
    {
        if(Interface::laser_scan[i].range)
        {
            r = Interface::laser_scan[i].range;
            th = NormalizeRadian(Interface::laser_scan[i].angle + pose[2]);

            Vector3d laser_point;
            laser_point[0] = r * std::cos(th) + pose[0];
            laser_point[1] = r * std::sin(th) + pose[1];
            laser_point[2] = pose[2];

            if(laser_point[0] <= 100 + Constants::ROBOT_CENTER_OFFSET) continue;
            if(laser_point[0] >= Constants::MapBorder[0] + Constants::ROBOT_CENTER_OFFSET + Constants::LIDAR_DATA_CUT_SIZE) continue;
            if(laser_point[1] <= -Constants::MapBorder[1] / 2.0 - Constants::LIDAR_DATA_CUT_SIZE) continue;
            if(laser_point[1] >= Constants::MapBorder[1] / 2.0 + Constants::LIDAR_DATA_CUT_SIZE) continue;

            Vector2i grid_pose = ConvertToGridCoordinate(laser_point);
            if (IsSafe(grid_pose, Constants::MAP_SIZE) && map_[grid_pose[0]][grid_pose[1]] < 128)
                weight += 1 - (map_[grid_pose[0]][grid_pose[1]] / 128.0);
            availableDataCnt++;
        }
    }

    if(availableDataCnt > 0)
        weight = weight/availableDataCnt;
    return weight;
}
void MCL::ComputeParticlesWeight()
{
    double weight;

    for(int i = 0; i < particle_max_; i++)
    {
        weight = ComputeWeightFromPose(particles_[i].pose);
        particles_[i].weight = weight;
    }
}

void MCL::Resampling()
{
    double total_weight = 0;
    std::array<double, particle_max_> particle_weights;
    Particle particle_sampled[particle_max_];

    particle_weights.data();
    double *weight_ptr = particle_weights.data();
    for (int i = 0; i < particle_max_; i++)
    {
        total_weight += particles_[i].weight;
        weight_ptr[i] = total_weight;
    }

    std::uniform_real_distribution<double> dart(0, total_weight);
    for (int i = 0; i < particle_max_; i++)
    {
        double darted = dart(generator_);
        auto lower_bound = std::lower_bound(particle_weights.begin(), particle_weights.end(), darted);
        int particle_idx = lower_bound - particle_weights.begin();

        particle_sampled[i] = particles_[particle_idx];
    }
    for (int i = 0; i < particle_max_; i++)
        particles_[i] = particle_sampled[i];
}
void MCL::OdometryMCLFusion()
{
	double total_pose[2] = { 0, }, deg_pos[2] = { 0, };
    double total_weight = 0;
    int availableDataCnt = 0;
    
    double percent = 0.4;
    if(DRIVE_TYPE == Tire_Drive || DRIVE_TYPE == X_Drive || DRIVE_TYPE == Mecanum_Drive) percent = 0.2;

    availableDataCnt = 0;
    for(int i = 0; i < particle_max_; i++)
    {
        if(particles_[i].weight > percent)
        {
            total_pose[0] += particles_[i].pose[0];
            total_pose[1] += particles_[i].pose[1];
            deg_pos[0] += std::cos(particles_[i].pose[2]);
            deg_pos[1] += std::sin(particles_[i].pose[2]);
            availableDataCnt++;
            total_weight += particles_[i].weight;
        }
    }

    if(availableDataCnt == 0)
    {
        for(int i = 0; i < particle_max_; i++)
        {
            total_pose[0] += particles_[i].pose[0];
            total_pose[1] += particles_[i].pose[1];
            deg_pos[0] += std::cos(particles_[i].pose[2]);
            deg_pos[1] += std::sin(particles_[i].pose[2]);
            availableDataCnt++;
            total_weight += particles_[i].weight;
        }
    }

    if(availableDataCnt > 0)
    {
        mcl_pose_[0] = total_pose[0] / availableDataCnt;
        mcl_pose_[1] = total_pose[1] / availableDataCnt;
        mcl_pose_[2] = std::atan2(deg_pos[0], deg_pos[1]);
    }
    else
    {
        return;
    }
    
    Interface::RobotPose[0] = mcl_pose_[0];
    Interface::RobotPose[1] = mcl_pose_[1];
    Interface::RobotPose[2] = mcl_pose_[2];
}
void MCL::Show(cv::Mat &mat)
{
    if (!initialized_)
        return;

    int x1, y1, x2, y2, r = 6;
    double th;
    for (int i = 0; i < particle_max_; i++)
    {
        Vector2i grid_pose = ConvertToGridCoordinate(particles_[i].pose);
        th = particles_[i].pose[2];

        x1 = grid_pose[0];
        y1 = grid_pose[1];
        x2 = x1 - r * std::cos(th);
        y2 = y1 + r * std::sin(th);
        cv::arrowedLine(mat, cv::Point(y1, x1), cv::Point(y2, x2), cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0, 0.4);
    }
}