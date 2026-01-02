// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModuleConfig.h"

/// @brief This is the same for all swerve modules in the system.
const std::string SwerveModuleConfig::canBus = "rio";

SwerveModuleConfig::SwerveModuleConfig() : 
_number(-1), 
_position(frc::Translation2d(units::meter_t(0.0) , units::meter_t(0.0))), 
_driveMotorId(-1), 
_steerMotorId(-1), 
_steerEncoderId(-1) {

}


bool SwerveModuleConfig::is_valid() const {
    if (_number < 0) return false;
    if (_driveMotorId < 0) return false;
    if (_steerMotorId < 0) return false;
    if (_steerEncoderId < 0) return false;

    return true;
}

