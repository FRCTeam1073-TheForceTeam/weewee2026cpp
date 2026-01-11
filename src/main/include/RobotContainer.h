// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include <frc/smartdashboard/SendableChooser.h>
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/OI.h"
#include "commands/TeleopDrive.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  // frc2::CommandPtr GetAutonomousCommand();
  bool haveInitStartPos;

  void autonomousInit();

  Command GetAutonomousCommand();
  Command GetTeleopCommand();
  Command GetDisbaledCommand();

  void disblaedInit();

  bool FindStartPos();

  bool DisabledPeriodic();


 private:
  const Drivetrain m_drivetrain;
  const OI m_OI;

  const TeleopDrive cmd_teleopDrive;

  bool isRed;

  const frc::SendableChooser<std::string> m_positionChooser;
  const std::string noPosition;
  const std::string rightPosition;
  const std::string leftPosition;
  const std::string centerPosition;

  const frc::SendableChooser<std::string> m_levelChooser;
  const std::string testAuto;

  void ConfigureBindings();
};
