// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include <frc/smartdashboard/SendableChooser.h>
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

  void GetAutonomousCommand();

  void AutonomousPeriodic();

  frc2::Command GetTeleopCommand();
  frc2::Command GetDisbaledCommand();

  void disblaedInit();

  bool FindStartPos();

  bool DisabledPeriodic();

 private:

  std::shared_ptr<Drivetrain> m_drivetrain;
  std::shared_ptr<OI> m_OI;

  std::shared_ptr<TeleopDrive> cmd_teleopDrive;

  bool isRed;

  void ConfigureBindings();

  double GearRatio = units::angle::turn_t(1)/units::angle::turn_t(1);
};
