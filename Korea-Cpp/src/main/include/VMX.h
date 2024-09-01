#pragma once

#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <studica/TitanQuad.h>
#include <studica/MockDS.h>

#include <AHRS.h>

#include "Constants.h"

class VMX
{
public:
    VMX();
    ~VMX();
    void Update();
private:
    studica::TitanQuad *motor_[4];
    studica::MockDS ds_;

    frc::DigitalInput *control_switch_[4];
    frc::DigitalOutput *control_led_[2];
    
    AHRS *ahrs_;

    bool control_switch_pressed_[3] = {false, false, false};

    int FindTitanNumber(int num);
    void GetControlSwitch();

    void GetGyro();

    void SetLED();

    void ComputeDriveSpeed();
    void SetMotorSpeed();
};