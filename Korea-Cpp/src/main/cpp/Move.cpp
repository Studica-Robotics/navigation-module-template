#include "Move.h"
#include "Interface.h"
#include "Util.h"

int Move::prev_filter_time_ = 0;

void Move::Stop()
{
    Interface::Velocity[0] = Interface::Velocity[1] = Interface::Velocity[2] = 0;
    prev_filter_time_ = 0;
}
void Move::filter(int x, int y, int w, bool acc_flg)
{
    int current_time;
    double acc[3] = {Constants::ACCELATION_VALUE[0], Constants::ACCELATION_VALUE[0], Constants::ACCELATION_VALUE[1]};
    int target_speed[3] = {x, y, w};

    current_time = GetTime();

    for (int i = 0; i < 3; i++)
    {
        if (std::abs(target_speed[i]) > std::abs(Interface::Velocity[i]))
        {
            acc_flg = true;
            break;
        }
    }

    if(!acc_flg)
    {
        Interface::Velocity[0] = target_speed[0];
        Interface::Velocity[1] = target_speed[1];
        Interface::Velocity[2] = target_speed[2];
        
        prev_filter_time_ = current_time;
        return;
    }

    if(prev_filter_time_ == 0 || (Interface::Velocity[0] == 0 && Interface::Velocity[1] == 0 && Interface::Velocity[2] == 0))
        prev_filter_time_ = current_time;

    if(current_time - prev_filter_time_ == 0)
    {
        acc[0] *= 0.01;
        acc[1] *= 0.01;
        acc[2] *= 0.01;
    } 
    else
    {
        acc[0] *= (current_time - prev_filter_time_) / 1000.0;
        acc[1] *= (current_time - prev_filter_time_) / 1000.0;
        acc[2] *= (current_time - prev_filter_time_) / 1000.0;
    }
    prev_filter_time_ = current_time;

    Interface::Autonomous = true;
    for (int i = 0; i < 3; i++)
    {
        if (std::abs(target_speed[i] - Interface::Velocity[i]) < acc[i])
            Interface::Velocity[i] = target_speed[i];
        else if (target_speed[i] > Interface::Velocity[i])
            Interface::Velocity[i] += acc[i];
        else if (target_speed[i] < Interface::Velocity[i])
            Interface::Velocity[i] -= acc[i];
    }
}