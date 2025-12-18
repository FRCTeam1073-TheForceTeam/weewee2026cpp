// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(SwerveModuleConfig cfg, SwerveModuleIDConfig ids):
steerMotor(ids.steerMotorID, "rio"), driveMotor(ids.driveMotorID, "rio"), position(cfg.posit1ion){
    //This is a comment
    //This is also a comment
    
}

// This method will be called once per scheduler run

