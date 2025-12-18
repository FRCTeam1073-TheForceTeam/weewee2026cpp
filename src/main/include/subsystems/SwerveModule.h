// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "SwerveModuleConfig.h"
#include "SwerveModuleIDConfig.h"
//#include "frc/motorcontrol/PWMMotorController.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/geometry/Translation2d.h>
#include <string.h>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>


using namespace ctre::phoenix6::hardware;


class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule(SwerveModuleConfig cfg, SwerveModuleIDConfig ids);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

  SwerveModuleConfig cfg;
  SwerveModuleIDConfig idcfg;
  TalonFX steerMotor;
  TalonFX driveMotor;
  frc::Translation2d position;
  VelocityVoltage driveVelocityVoltage;
  PositionVoltage steerPositionVoltage;
  double targetSteerRotations;
  double targetDriveVelocity;
  double targetDriveVelocityRotations;
  double steerVelocity;
  const std::string kCANbus = "rio";
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
