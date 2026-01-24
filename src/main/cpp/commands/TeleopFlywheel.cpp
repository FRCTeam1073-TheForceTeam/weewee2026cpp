// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"
#include "subsystems/Flywheel.h"

Flywheel::Flywheel(): 
    _hardwareConfigured(true), 
    _leadFlywheelMotor(LeadMotorId, "rio"),
    _followFlywheelMotor(FollowMotorId, "rio"),
    _FlywheelVelocitySig(_leadFlywheelMotor.GetVelocity()),
    _FlywheelCurrentSig(_leadFlywheelMotor.GetTorqueCurrent()),
    _FlywheelVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)) {
        
    _FlywheelVelocityVoltage.WithSlot(0);

    }
void Flywheel::SetFlywheelVelocity(units::angular_velocity::turns_per_second_t Velocity) {
    _TargetVelocity = Velocity;
}
Shoot::Shoot(Flywheel* flywheel)
    : m_flywheel{flywheel} {
      SetFlywheelVelocity(1.0); //TODO: make this number correct
  // Register that this command requires the subsystem.
  AddRequirements(m_flywheel);
}
