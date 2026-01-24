// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/IntakeCollector.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
IntakeCollector::IntakeCollector() :
_hardwareConfigured(true),
_intakeMotor(MotorId, CANBus("rio")),
_IntakeVelocitySig(_intakeMotor.GetVelocity()),
_IntakeCurrentSig(_intakeMotor.GetTorqueCurrent()),
_commandVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)) {
  // Extra implementation of subsystem constructor goes here.

  // Assign gain slots for the commands to use:
  _commandVelocityVoltage.WithSlot(0);  // Velocity control loop uses these gains.

  _targetVelocity = 0_tps;

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "IntakeCollector: Hardware Failed To Configure!" << std::endl;
  }

}


  /// Set the command for the system.
void IntakeCollector::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void IntakeCollector::SetIntakeVelocity(units::angular_velocity::turns_per_second_t Velocity) {
  _targetVelocity = Velocity;
}

ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> IntakeCollector::GetIntakeVelocity() {
  return _IntakeVelocitySig;
}

units::angular_velocity::turns_per_second_t IntakeCollector::GetIntakeTargetVelocity() {
  return _targetVelocity;
}


void IntakeCollector::Periodic() {
  // Sample the hardware:
  BaseStatusSignal::RefreshAll(_IntakeVelocitySig, _IntakeCurrentSig);

  // Populate feedback cache:
  _feedback.force = _IntakeCurrentSig.GetValue() / AmpsPerNewton; // Convert from hardware units to subsystem units.
  _feedback.velocity = _IntakeVelocitySig.GetValue() / TurnsPerMeter; // Convert from hardare units to subsystem units.

  _intakeMotor.Set(_targetVelocity.value());

  // // Process command:
  if (std::holds_alternative<units::velocity::meters_per_second_t>(_command)) {
      // Send velocity based command:

      // Convert to hardware units:
      // Multiply by conversion to produce commands.
      auto angular_vel = std::get<units::velocity::meters_per_second_t>(_command) * TurnsPerMeter;
      // Send to hardware:
      _intakeMotor.SetControl(_commandVelocityVoltage.WithVelocity(angular_vel));
  } else if (std::holds_alternative<units::length::meter_t>(_command)) {
      // Send position based command:

      // Convert to hardware units:
      auto angle = std::get<units::length::meter_t>(_command) * TurnsPerMeter;

  } else {
      // No command, so send a "null" neutral output command if there is no position or velocity provided as a command:
    _intakeMotor.SetControl(controls::NeutralOut());
  }
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool IntakeCollector::ConfigureHardware() {
configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; // Set current limits to keep from breaking things.
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; 

    configs.Voltage.PeakForwardVoltage = 8_V; // These are pretty typical values, adjust as needed.
    configs.Voltage.PeakReverseVoltage = -8_V;

    // Slot 0 for the velocity control loop:
    configs.Slot0.kV = 0.12;
    configs.Slot0.kP = 0.15;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.01;
    configs.Slot0.kA = 0.0;
    // Slot 1 for position control mode:
    configs.Slot1.kV = 0.12; // Motor constant.
    configs.Slot1.kP = 0.1;
    configs.Slot1.kI = 0.01;
    configs.Slot1.kD = 0.0;
    configs.Slot1.kA = 0.0;

    // Set whether motor control direction is inverted or not:
    configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

    // Set the control configuration for the drive motor:
    auto status = _intakeMotor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        std::cerr << "IntakeCollector: Configuration went wrong" << std::endl;
    }

    // Set our neutral mode to brake on:
    status = _intakeMotor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

    if (!status.IsOK()) {
        std::cerr << "IntakeCollector: Neutral brake went wrong" << std::endl;
    }
    //TODO: change error messages if they are incorrect which they probably are

    // Depends on mechanism/subsystem design:
    // Optionally start out at zero after initialization:
    _intakeMotor.SetPosition(units::angle::turn_t(0));

    // Log errors.
    return false;

}
