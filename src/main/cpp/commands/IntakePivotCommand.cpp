
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakePivotCommand.h"

ExampleCommand::ExampleCommand(IntakePivot* intakepivot)
    : m_intakePivot{intakepivot} {
  // Register that this command requires the subsystem.
  AddRequirements(m_intakePivot);
}
