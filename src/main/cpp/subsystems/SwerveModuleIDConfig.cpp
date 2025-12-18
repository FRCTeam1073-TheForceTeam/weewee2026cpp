// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModuleIDConfig.h"

SwerveModuleIDConfig::SwerveModuleIDConfig(){
    driveMotorID = 0;
    steerMotorID = 0;
    steerEncoderID = 0;
}

SwerveModuleIDConfig::SwerveModuleIDConfig(int drive, int steer, int encoder){
    driveMotorID = drive;
    steerMotorID = steer;
    steerEncoderID = encoder;
}

// This method will be called once per scheduler run

