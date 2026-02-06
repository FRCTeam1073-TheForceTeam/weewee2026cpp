// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/TeleopDrive.h"
#include "commands/Collect.h"
#include "subsystems/LaserCan.h"

const std::string RobotContainer::noPosition = "No Position";
const std::string RobotContainer::rightPosition = "Right Auto";
const std::string RobotContainer::leftPosition = "Left Auto";
const std::string RobotContainer::centerPosition = "Center Auto";
const std::string RobotContainer::testAuto = "Test Auto";


RobotContainer::RobotContainer() {
  m_drivetrain = std::make_shared<Drivetrain>();
  m_OI = std::make_shared<OI>();
  m_flywheel = std::make_shared<Flywheel>();
  m_intake = std::make_shared<Intake>();
  m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI));
  cmd_collect = std::make_shared<Collect>(m_intake);
  m_shooterLoad = std::make_shared<ShooterLoad>();
  m_climber = std::make_shared<Climber>();
<<<<<<< HEAD
  m_flywheel->SetDefaultCommand(FlywheelTeleop(m_flywheel));
  m_climber->SetDefaultCommand(ClimberTeleop(m_climber));
  m_intake->SetDefaultCommand(IntakeTeleop(m_intake));
  m_shooterHood->SetDefaultCommand(HoodTeleop(m_shooterHood));
  m_shooterLoad->SetDefaultCommand(LoaderTeleop(m_shooterLoad));
  m_spindexer->SetDefaultCommand(SpindexerTeleop(m_spindexer));
  //m_laser = std::make_shared<LaserCan>();
=======
  m_laser = std::make_shared<LaserCan>();
>>>>>>> 5d36825946a2d4bb0ff9f39f7d9786d483d72eae

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
}



// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//  // TODO:
// }
