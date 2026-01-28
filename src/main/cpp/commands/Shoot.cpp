// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"

Shoot::Shoot(std::shared_ptr<Flywheel> flywheel) :
  m_flywheel{flywheel} {

  AddRequirements({m_flywheel.get()});
}

// Called wh(std::shared_ptr<Drivetrain> drivetrainen the command is initially scheduled.
void Shoot::Initialize() {
  m_flywheel->SetVelocity(1.0_tps);//TODO: change number into one that catually makes sense
}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() {}

// Called once the command ends or is interrupted.
void Shoot::End(bool interrupted) {
  m_flywheel->SetVelocity(0.0_tps);//TODO: change number into one that catually makes sense
}

// Returns true when the command should end.
bool Shoot::IsFinished() {
  return false;
}
