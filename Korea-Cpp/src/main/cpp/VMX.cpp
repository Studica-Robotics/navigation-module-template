#include <array>

#include "VMX.h"
#include "Interface.h"
#include "Util.h"

VMX::VMX()
{
    for(int i = 0; i < 4; i++)
        if(Constants::VMX_SWITH[i])
            control_switch_[i] = new frc::DigitalInput(Constants::VMX_SWITH[i]);

    control_led_[0] = new frc::DigitalOutput(Constants::LED_GREEN);
    control_led_[1] = new frc::DigitalOutput(Constants::LED_RED);

    ahrs_ = new AHRS(frc::SPI::Port::kMXP, 200);

    motor_[0] = new studica::TitanQuad(42, 20000, Constants::MOTOR[0]);
    motor_[1] = new studica::TitanQuad(42, 20000, Constants::MOTOR[1]);
    motor_[2] = new studica::TitanQuad(42, 20000, Constants::MOTOR[2]);
    motor_[3] = new studica::TitanQuad(42, 20000, Constants::MOTOR[3]);

    ds_.Enable();
}

VMX::~VMX()
{
    for(int i = 0; i < 4; i++)
        if(Constants::VMX_SWITH[i])
            delete control_switch_[i];

    delete control_led_[0];
    delete control_led_[1];

    delete ahrs_;

    delete motor_[0];
    delete motor_[1];
    delete motor_[2];
    delete motor_[3];
}

void VMX::Update()
{
    SetLED(); 
    GetGyro();
    GetControlSwitch();
    SetMotorSpeed();
}

int VMX::FindTitanNumber(int num)
{
    for (int i = 0; i < 4; i++)
        if (num == Constants::MOTOR[i])
            return i;
    return 0;
}
void VMX::GetControlSwitch()
{   
    bool *switch_ptr[4] = {&Interface::EMS, &Interface::SW1, &Interface::SW2, &Interface::SW3};

    for(int i = 0; i < 4; i++)
    {
        bool value = false;

        if(Constants::VMX_SWITH[i])
            value = control_switch_[i]->Get();
        else if(Constants::TITAN_SWITCH[i][0] != -1)
            value = motor_[FindTitanNumber(Constants::TITAN_SWITCH[i][0])]->GetLimitSwitch(Constants::TITAN_SWITCH[i][1]);

        *switch_ptr[i] = Constants::SWITCH_INV[i] ? value : !value;
    }

    if(!control_switch_pressed_[0] && Interface::SW1) control_switch_pressed_[0] = true;
    else if(control_switch_pressed_[0] && !Interface::SW1)
    {
        control_switch_pressed_[0] = false;
        Interface::SW1_Pressed = true;
    }
    if(!control_switch_pressed_[1] && Interface::SW2) control_switch_pressed_[1] = true;
    else if(control_switch_pressed_[1] && !Interface::SW2)
    {
        control_switch_pressed_[1] = false;
        Interface::SW2_Pressed = true;
    }
    if(!control_switch_pressed_[2] && Interface::SW3) control_switch_pressed_[2] = true;
    else if(control_switch_pressed_[2] && !Interface::SW3)
    {
        control_switch_pressed_[2] = false;
        Interface::SW3_Pressed = true;
    }
    if(Interface::EMS) Interface::SW1_Pressed = Interface::SW2_Pressed = Interface::SW3_Pressed = false;
}
void VMX::GetGyro()
{
    static double prev_yaw = 999;
    double n_yaw;

    if(Interface::EMS)
    {
        Interface::GyroReset = true;
        return;
    }
    
    if(!Interface::EMS && Interface::GyroReset)
    {
        prev_yaw = 999;
        Interface::GyroReset = false;
    }

    n_yaw = ToRadian(ahrs_->GetYaw());
    if (prev_yaw == 999) prev_yaw = n_yaw;
    Interface::GyroYaw = NormalizeRadian(n_yaw - prev_yaw);
}
void VMX::SetLED()
{
    control_led_[0]->Set(Interface::LED_G);
    control_led_[1]->Set(Interface::LED_R);
}
void VMX::ComputeDriveSpeed()
{
    if(Interface::EMS)
    {
        Interface::Velocity[0] = Interface::Velocity[1] = Interface::Velocity[2] = 0;
        return;
    }

    double motor_speed[4], max_speed = 0, rpm[3], err[2] = {0.87, 1};
    
    rpm[0] = (Interface::Velocity[0] * 60) / Constants::WHEEL_CIRCUMFERENCE;
    rpm[1] = (Interface::Velocity[1] * 60) / Constants::WHEEL_CIRCUMFERENCE;
    rpm[2] = (ToRadian(Interface::Velocity[2]) * 30 * Constants::WHEEL_WIDTH) / Constants::WHEEL_CIRCUMFERENCE;

    switch (DRIVE_TYPE)
    {
    case X_Drive:
        motor_speed[0] = (rpm[0] + rpm[1]) / std::sqrt(2) + rpm[2];
        motor_speed[1] = (-rpm[0] + rpm[1]) / std::sqrt(2) + rpm[2];
        motor_speed[2] = (rpm[0] - rpm[1]) / std::sqrt(2) + rpm[2];
        motor_speed[3] = (-rpm[0] - rpm[1]) / std::sqrt(2) + rpm[2];
        break;
    case Mecanum_Drive:
        motor_speed[0] = rpm[0] * err[0] + rpm[1] + rpm[2];
        motor_speed[1] = -rpm[0] * err[1] + rpm[1] + rpm[2];
        motor_speed[2] = rpm[0] * err[0] - rpm[1] + rpm[2];
        motor_speed[3] = -rpm[0] * err[1] - rpm[1] + rpm[2];
        break;
    case Diff_Drive:
        motor_speed[0] = rpm[0] + rpm[2];
        motor_speed[1] = -rpm[0] + rpm[2];
        motor_speed[2] = 0;
        motor_speed[3] = 0;
        break;
    case Tire_Drive:
    case SixWheel_Drive:
        motor_speed[0] = rpm[0] + rpm[2];
        motor_speed[1] = -rpm[0] + rpm[2];
        motor_speed[2] = motor_speed[0];
        motor_speed[3] = motor_speed[1];
        break;
    case Omni3_Drive:
        motor_speed[0] = rpm[0] * std::sqrt(3) / 2 + rpm[1] / 2 + rpm[2];
        motor_speed[1] = -rpm[0] * std::sqrt(3) / 2 + rpm[1] / 2 + rpm[2];
        motor_speed[2] = - rpm[1] + rpm[2];
        motor_speed[3] = 0;
        break;
    }

    if(std::abs(motor_speed[0]) > std::abs(max_speed)) max_speed = std::abs(motor_speed[0]);
    if(std::abs(motor_speed[1]) > std::abs(max_speed)) max_speed = std::abs(motor_speed[1]);
    if(std::abs(motor_speed[2]) > std::abs(max_speed)) max_speed = std::abs(motor_speed[2]);
    if(std::abs(motor_speed[3]) > std::abs(max_speed)) max_speed = std::abs(motor_speed[3]);

    if (max_speed > Constants::RPM)
    {
        motor_speed[0] /= max_speed / Constants::RPM;
        motor_speed[1] /= max_speed / Constants::RPM;
        motor_speed[2] /= max_speed / Constants::RPM;
        motor_speed[3] /= max_speed / Constants::RPM;
    }
    
    Interface::Motor[0] = motor_speed[0] / double(Constants::RPM);
    Interface::Motor[1] = motor_speed[1] / double(Constants::RPM);
    Interface::Motor[2] = motor_speed[2] / double(Constants::RPM);
    Interface::Motor[3] = motor_speed[3] / double(Constants::RPM);
}
void VMX::SetMotorSpeed()
{
    if(!Interface::SignalTest) ComputeDriveSpeed();
    for (int i = 0; i < 4; i++)
    {
        if(Interface::EMS) Interface::Motor[i] = 0;
        motor_[i]->Set(Constants::MOTOR_INV[i] ? -Interface::Motor[i] : Interface::Motor[i]);
    }
}