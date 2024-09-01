#pragma once

#include <frc/RobotBase.h>
#include <atomic>
#include <thread>

class Robot : public frc::RobotBase
{
public:
    void StartCompetition() override;
    void EndCompetition() override;

private:
    std::atomic<bool> m_exit{false};
    std::thread thread_object_;

    void VMXThread();
};
