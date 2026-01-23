// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/force.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <variant>

class ShooterActivate : public frc2::SubsystemBase {
 public:

 struct ShooterActivateFeedback {
      units::velocity::meters_per_second_t velocity; // TODO: Add other stuff to feedback
      units::force::newton_t force;
  };

  using Command = std::variant<std::monostate, units::velocity::meters_per_second_t, units::length::meter_t>;

  ShooterActivate();

  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  
  const ShooterActivateFeedback& GetShooterActivateFeedback() const { return _feedback; }

 private:
  static constexpr int MotorId = 9; // TODO: Get motor id 

  const double GearRatio = units::angle::turn_t(1)/units::angle::turn_t(1); // TODO: Get gear ratio from EM

  

  // Mechanism conversion constants for the subsystem:
  static constexpr auto TurnsPerMeter = units::angle::turn_t(32.0) / units::length::meter_t(1.0); // TODO: Get turns per meter
  static constexpr auto AmpsPerNewton = units::current::ampere_t(10.0) / units::force::newton_t(1.0); // TODO: Get amps per newton

  
  //  TalonFX motor interface.
  ctre::phoenix6::hardware::TalonFX _intakeMotor;

  // CTRE hardware feedback signals:
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _IntakeVelocitySig;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> _IntakeCurrentSig;


  //  velocity and position controls:
  ctre::phoenix6::controls::VelocityVoltage _commandVelocityVoltage;  // Uses Slot0 gains.
  
  // Cached feedback:
  ShooterActivateFeedback _feedback;

  // Cached command: Variant of possible different kinds of commands.
  Command  _command;

  // Set the motors target velocity
  units::angular_velocity::turns_per_second_t _targetVelocity;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
