// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/Collector.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
Subsystem::Subsystem() :
_hardwareConfigured(true),
_Motor(MotorId, CANBus("rio")),
_PositionSig(_Motor.GetPosition()),
_VelocitySig(_Motor.GetVelocity()),
_CurrentSig(_Motor.GetTorqueCurrent()),
_commandVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)),
_commandPositionVoltage(units::angle::turn_t(0.0)) {
  // Extra implementation of subsystem constructor goes here.

  // Assign gain slots for the commands to use:
  _commandVelocityVoltage.WithSlot(0);  // Velocity control loop uses these gains.
  _commandPositionVoltage.WithSlot(1);  // Position control loop uses these gains.

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "Subsystem: Hardware Failed To Configure!" << std::endl;
  }

}


  /// Set the command for the system.
void Subsystem::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}


void Subsystem::Periodic() {
  // Sample the hardware:
  BaseStatusSignal::RefreshAll(_PositionSig, _VelocitySig, _CurrentSig);

  // Latency compensate the feedback when you sample a value and its rate:
  auto compensatedPos = BaseStatusSignal::GetLatencyCompensatedValue(_PositionSig, _VelocitySig);

  // Populate feedback cache:
  _feedback.force = _CurrentSig.GetValue() / AmpsPerNewton; // Convert from hardware units to subsystem units.
  _feedback.position = compensatedPos / TurnsPerMeter; // Convert from hardare units to subsystem units. Divide by conversion to produce feedback.
  _feedback.velocity = _VelocitySig.GetValue() / TurnsPerMeter; // Convert from hardare units to subsystem units.


  // // Process command:
  if (std::holds_alternative<units::velocity::meters_per_second_t>(_command)) {
      // Send velocity based command:

      // Convert to hardware units:
      // Multiply by conversion to produce commands.
      auto angular_vel = std::get<units::velocity::meters_per_second_t>(_command) * TurnsPerMeter;
      // Send to hardware:
      _Motor.SetControl(_commandVelocityVoltage.WithVelocity(angular_vel));
  } else if (std::holds_alternative<units::length::meter_t>(_command)) {
      // Send position based command:

      // Convert to hardware units:
      auto angle = std::get<units::length::meter_t>(_command) * TurnsPerMeter;

      // Send to hardware:
      _Motor.SetControl(_commandPositionVoltage.WithPosition(angle));
  } else {
      // No command, so send a "null" neutral output command if there is no position or velocity provided as a command:
    _Motor.SetControl(controls::NeutralOut());
  }
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool Subsystem::ConfigureHardware() {
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
    auto status = _Motor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        // Log errors.
    }

    // Set our neutral mode to brake on:
    status = _Motor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

    if (!status.IsOK()) {
        // Log errors.
    }


    // Depends on mechanism/subsystem design:
    // Optionally start out at zero after initialization:
    _Motor.SetPosition(units::angle::turn_t(0));

    // Log errors.
    return false;

}
