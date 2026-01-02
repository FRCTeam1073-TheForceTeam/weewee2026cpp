// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/geometry/Translation2d.h>
#include <cmath>

/**  @brief This class provides the configuration specific to indivudal swerve modules and
 *  associated helper functions.
 * 
 * This configuration is typically fixed at runtime matching wiring and physical
 * configuration of the modules and CAN ID assignment in firmware.
*/
class SwerveModuleConfig {
 public:
  SwerveModuleConfig();

  // Fluent builder for setting ID.
  SwerveModuleConfig & with_id(int num) {
    _number = num;
    return *this;
  }

  // Fluent builder for setting position.
  SwerveModuleConfig & with_position(const frc::Translation2d& pos) {
    _position = pos;
    return *this;
  }


  SwerveModuleConfig & with_drive_motor_id(int id) {
    _driveMotorId = id;
    return *this;
  }

  SwerveModuleConfig & with_steer_motor_id(int id) {
    _steerMotorId = id;
    return *this;
  }

  SwerveModuleConfig & with_steer_encoder_id(int id) {
    _steerEncoderId = id;
    return *this;
  }

  static const std::string canBus;   /// Every module is on a the same CANBus in a robot.

  /**
   * Returns true if this is a valid swerve module config (all fields set).
   */
  bool is_valid() const;

  /// @brief  Return which module number this is.
  int number() const { return _number;}

  /// @brief  Return the position of the module center within the robot.
  const frc::Translation2d& position() { return _position; }

  /// @brief Return the CANBus ID for the drive motor of the module.
  int drive_motor_id() { return _driveMotorId; }

  /// @brief Return the CANBus ID for the steering motor of the module.
  int steer_motor_id() { return _steerMotorId; }

  /// @brief Return the CANBus ID for the steering encoder on the module.
  int steer_encoder_id() { return _steerEncoderId; }

private:

  int _number;                  /// Every module has a unique numnber 0, through ...N (typically 4).
  frc::Translation2d _position; /// Every module has a unique position within the robot relative to robot center in hardware.
  int _driveMotorId;            /// Every module has a unique drive motor CAN ID in firmware
  int _steerMotorId;            /// Every module has a unique steer motor CAN ID in firmware
  int _steerEncoderId;          /// Every module has a unique encoder CAN ID in firmware
};
