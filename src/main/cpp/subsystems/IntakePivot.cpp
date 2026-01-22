// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakePivot.h"
#include <iostream>

using namespace ctre::phoenix6;



IntakePivot::IntakePivot() : 
    _hardwareConfigured(true),
    _IntakePivotMotor(67, CANBus("rio")),
    _IntakePivotEncoder(41, CANBus("rio"))
{
// _hardwareConfigured = ConfigureHardware();
//   if (!_hardwareConfigured) {
//     std::cerr << "ExampleSubsystem: Hardware Failed To Configure!" << std::endl;
//   }  
}