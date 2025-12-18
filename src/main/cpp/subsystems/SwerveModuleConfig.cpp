// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModuleConfig.h"

SwerveModuleConfig::SwerveModuleConfig():position(frc::Translation2d(units::meter_t(0.0) , units::meter_t(0.0))){
    steerP = 0;
    steerI = 0;
    steerD = 0;
    steerV = 0;
    driveP = 0;
    driveI = 0;
    driveD = 0;
    driveV = 0;
    driveA = 0;
    moduleNumber = -1;

    SwerveModuleConfig();{
        driveP = 0.3;
        driveI = 0.0;
        driveD = 0.001;
        driveV = 0.12;
        driveA = 0.05;
        steerP = 30.0;
        steerI = 4.0;
        steerD = 1.0;
        steerV = 0.12;
    }

}
