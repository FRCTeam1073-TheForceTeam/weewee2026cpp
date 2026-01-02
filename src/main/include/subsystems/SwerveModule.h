// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/current.h>
#include "SwerveModuleConfig.h"
#include "SwerveControlConfig.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/geometry/Translation2d.h>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>

#include <string>

/**
 * There is one SwerveModule object instance for each hardware swerve module in the drivetrain.
 * 
 * This class encapsulates the hardware configuration and interface of each module behind a 
 * simple API with physical units that is used by the top-level Drivetrain class to manage
 * the whole drivetrain.
 */
class SwerveModule {
 public:

  /**
   * Strongly typed swerve module state feedback with units.
   */
  class State {
    public:
      State() = default;
      units::time::microsecond_t time_stamp = 0.0_us;
      units::angle::radian_t steering_angle = 0.0_rad;
      units::angular_velocity::radians_per_second_t steering_velocity = 0.0_rad_per_s;
      units::velocity::meters_per_second_t drive_velocity = 0.0_m/1.0_s;
      units::length::meter_t drive_position = 0.0_m;
      units::current::ampere_t drive_current = 0.0_A; // Allows approximate effort computation and direction of force.
  };

  /**
   * Strongly typed swerve module command with units.
   */
  class Command {
    public:
      Command() = default;
      Command(units::time::microsecond_t now, units::angle::radian_t steer, units::velocity::meters_per_second_t drive) : 
        time_stamp(now), 
        steering_angle(steer),
        drive_velocity(drive) {};
      units::time::microsecond_t time_stamp = 0.0_us;
      units::angle::radian_t steering_angle = 0.0_rad;
      units::velocity::meters_per_second_t drive_velocity = 0.0_m/1.0_s;
  };

  SwerveModule(SwerveModuleConfig mod_cfg, std::shared_ptr<SwerveControlConfig> con_cfg);

  /// Configure the hardware for this module.
  bool configure_hardware();


  /// Access the swerve module config:
  const SwerveModuleConfig& module_config() const { _module_cfg;}

  /** Sample the hardware feedback information from the module, given the current timestamp which is store with the state.
   *  This avoids duplicating the sampling of time multiple times per update cycle.
   * 
   * This sampling implementation provides basic latency compensation for tighter control performance.
   * 
   */
  const State& sample(units::time::microsecond_t now);

  /// Read the current swerve modulestate:
  const State& state() const { return _latestState; }

  /// Update the commands to the hardware.
  void set_command(Command cmd);

 private:

  // Helper function for configuring drive hardware.
  bool configure_drive_hardware();

  // Helper function for configuring steering hardware.
  bool configure_steer_hardware();

  // Module-specific configuration:
  SwerveModuleConfig _module_cfg;

  // Shared controller-level configuration:
  std::shared_ptr<SwerveControlConfig> _control_cfg;

  // Hardware interfaces:
  ctre::phoenix6::hardware::TalonFX _steerMotor;
  ctre::phoenix6::hardware::TalonFX _driveMotor;
  ctre::phoenix6::hardware::CANcoder _steerEncoder;

  // Module-level controls:
  ctre::phoenix6::controls::VelocityVoltage _driveVelocityVoltage;
  ctre::phoenix6::controls::PositionVoltage _steerPositionVoltage;

  // CTRE hardware feedback signals:
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _steerVelocitySig;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> _steerPositionSig;

  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _driveVelocitySig;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> _drivePositionSig;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> _driveCurrentSig;

  // Cached state read from hardware during most recent sample() operation:
  State _latestState;

  // Cached last command sent from drivetrain:
  Command _latestCommand;

};
