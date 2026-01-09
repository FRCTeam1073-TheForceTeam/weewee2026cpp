// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/OI.h"

OI::OI() :
    _driverController = new frc::Joystick(0);
    _operatorController = new frc::Joystick(1);
    _driverAButton = new frc2::JoystickButton(_driverController, Buttons::AButton);
    _driverBButton = new frc2::JoystickButton(_driverController, Buttons::BButton);
    _driverXButton = new frc2::JoystickButton(_driverController, Buttons::XButton);
    _driverYButton = new frc2::JoystickButton(_driverController, Buttons::YButton);


// This method will be called once per scheduler run
void OI::Periodic() {}
