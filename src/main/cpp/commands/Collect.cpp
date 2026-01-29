// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Collect.h"

Collect::Collect(std::shared_ptr<IntakeCollector> IntakeCollector) :
  m_intakeCollector{IntakeCollector} {
{
AddRequirements({m_intakeCollector.get()});
}
  
}

// Called when the command is initially scheduled.
void Collect::Initialize() {
  m_intakeCollector->SetIntakeVelocity(1.0_tps);
}

// Called repeatedly when this Command is scheduled to run
void Collect::Execute() {
  m_intakeCollector->SetIntakeVelocity(1.0_tps);
}

// Called once the command ends or is interrupted.
void Collect::End(bool interrupted) {
  m_intakeCollector->SetIntakeVelocity(0.0_tps);
}

// Returns true when the command should end.
bool Collect::IsFinished() {
  return false;
}
