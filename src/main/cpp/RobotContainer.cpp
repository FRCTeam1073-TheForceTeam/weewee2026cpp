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
  m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI));

  // Configure the button bindings
  ConfigureBindings();
}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   return TestAuto::Create();
// }

void RobotContainer::autonomousInit() {
  trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Test_Auto");
}

void RobotContainer::AutonomousPeriodic() {
  auto sample = trajectory.value().SampleAt(frc::Timer::GetMatchTime());
  frc::ChassisSpeeds speeds = sample.value().GetChassisSpeeds();

  frc::SmartDashboard::PutNumber("Trajectory/Vx", speeds.vx());
  frc::SmartDashboard::PutNumber("Trajectory/Vy", speeds.vy());
  frc::SmartDashboard::PutNumber("Trajectory/Vw", speeds.omega());
  m_drivetrain->SetChassisSpeeds(speeds);
}

void RobotContainer::ConfigureBindings() {
}



// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//  // TODO:
// }
