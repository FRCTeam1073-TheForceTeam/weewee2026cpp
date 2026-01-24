// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "subsystems/ShooterActivate.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

ShooterActivate::ShooterActivate(): 
    _hardwareConfigured = true;

// This method will be called once per scheduler run
void ShooterActivate::Periodic() {}
