// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopFlywheel.h"

TeleopFlywheel::TeleopFlywheel(std::shared_ptr<Flywheel> flywheel) :
  m_flywheel{flywheel} {

  AddRequirements({m_flywheel.get()});
}

// Called wh(std::shared_ptr<Drivetrain> drivetrainen the command is initially scheduled.
void TeleopFlywheel::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleopFlywheel::Execute() {
  m_flywheel->SetVelocity(0.0_tps);
}

// Called once the command ends or is interrupted.
void TeleopFlywheel::End(bool interrupted) {}

// Returns true when the command should end.
bool TeleopFlywheel::IsFinished() {
  return false;
}
