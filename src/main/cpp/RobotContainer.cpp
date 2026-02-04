// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/TeleopDrive.h"
#include "commands/Autos/TestAuto.h"

const std::string RobotContainer::noPosition = "No Position";
const std::string RobotContainer::rightPosition = "Right Auto";
const std::string RobotContainer::leftPosition = "Left Auto";
const std::string RobotContainer::centerPosition = "Center Auto";
const std::string RobotContainer::testAuto = "Test Auto";

RobotContainer::RobotContainer() {
  m_drivetrain = std::make_shared<Drivetrain>();
  m_OI = std::make_shared<OI>();
  m_Tags = std::make_shared<AprilTagFinder>();
  m_FieldMap = std::make_shared<FieldMap>();
  m_Localizer = std::make_shared<Localizer>(m_drivetrain, m_Tags);
  m_FieldDisplay = std::make_shared<FieldMapDisplay>(m_drivetrain, m_Localizer, m_FieldMap);
  m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI, m_Localizer));

  trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Test_Auto");

  // Configure the button bindings
  ConfigureBindings();
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return TestAuto::Create(m_drivetrain, m_Localizer, trajectory);
}

void RobotContainer::autonomousInit() {
}

void RobotContainer::AutonomousPeriodic() {
}

void RobotContainer::ConfigureBindings() {
}
