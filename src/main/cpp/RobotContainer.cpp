// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  noPosition = "No Position",
  rightPosition = "Right Auto",
  leftPosition = "Left Auto",
  centerPosition = "Center Auto",
  testAuto = "Test Auto",
  cmd_teleopDrive(Drivetrain m_drivetrain, OI m_OI),
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
}



// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//  // TODO:
// }
