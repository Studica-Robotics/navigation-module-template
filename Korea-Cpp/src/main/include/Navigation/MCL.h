#pragma once

#include <random>
#include <opencv2/core/core.hpp>

#include "Interface.h"
#include "Util.h"

struct Particle
{
    double weight = 0;
    Vector3d pose;
};
class MCL
{
public:
    MCL(unsigned char **map);
    ~MCL();

    void Init();
    void Reset();
    void Update();
    void Show(cv::Mat &mat);

private:
    static const int particle_max_ = 100;
    
    unsigned char **map_;
    Particle *particles_;
    bool initialized_;
    std::default_random_engine generator_;
    Vector3d mcl_pose_;
    
    int prev_time_;

    void Prediction();
    double ComputeWeightFromPose(const Vector3d &pose);
    void ComputeParticlesWeight();
    void Resampling();
    void OdometryMCLFusion();

    void CalRobotPose(Vector3d &pose);
};