// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakePivot.h"
#include <iostream>

using namespace ctre::phoenix6;
using namespace ctre::phoenix;

const CANBus IntakePivot::canBus(CANBus::RoboRIO());

IntakePivot::IntakePivot() {
   _intakeCollectorMotor;
   _intakeCollectorCoder;
   ffGains;
   fbGains;
}

void IntakePivot::Periodic() {

}
