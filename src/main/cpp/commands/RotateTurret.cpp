// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateTurret.h"

//TODO: finish the command; it is not complete yet

RotateTurret::RotateTurret(std::shared_ptr<ShooterRotater> shooterRotater, std::shared_ptr<OI> oi) :
  m_shooterRotater {shooterRotater},
  m_OI{oi} {
  lastError = 0,
  isAlignedToHub = false,
  angularVel = 0_rad_per_s,
  position = 0_rad,//zeroed position is touching the hard stop
  AddRequirements({m_shooterRotater.get(), m_OI.get()});
}
  // Use addRequirements() here to declare subsystem dependencies.


// Called when the command is initially scheduled.
void RotateTurret::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RotateTurret::Execute() {}

// Called once the command ends or is interrupted.
void RotateTurret::End(bool interrupted) {}

// Returns true when the command should end.
bool RotateTurret::IsFinished() {
  return false;
}
