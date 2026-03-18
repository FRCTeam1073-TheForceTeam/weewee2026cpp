// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/button/Trigger.h>
#include "commands/TeleopDrive.h"

RobotContainer::RobotContainer() {
  m_drivetrain = std::make_shared<Drivetrain>();
  m_OI = std::make_shared<OI>();
  m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI));

  m_drivetrain->ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad)));

  // Configure the button bindings
  ConfigureBindings();
  
}

void RobotContainer::GetAutonomousCommand() {
}

void RobotContainer::autonomousInit() {
}

void RobotContainer::AutonomousPeriodic() {
}

void RobotContainer::ConfigureBindings() {
}
