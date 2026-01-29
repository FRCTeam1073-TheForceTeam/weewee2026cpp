// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos/TestAuto.h"

std::shared_ptr<Drivetrain> TestAuto::m_drivetrain = nullptr;

TestAuto::TestAuto(std::shared_ptr<Drivetrain> drivetrain)
{
  TestAuto::m_drivetrain = drivetrain;
  AddRequirements(m_drivetrain.get());
}

std::unique_ptr<frc2::Command> TestAuto::Create() {

  Path path = Path(std::vector<Path::Segment>(), std::numbers::pi * 1_rad, 2_mps); // TODO: path should be a segment of the trajectory

  return std::make_unique<frc2::SequentialCommandGroup>(DrivePath(TestAuto::m_drivetrain, path));
}
