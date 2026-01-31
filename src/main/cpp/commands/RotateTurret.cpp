// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateTurret.h"
#include <iostream>
#include <frc/DriverStation.h>

//TODO: finish the command; it is not complete yet

RotateTurret::RotateTurret(std::shared_ptr<ShooterRotater> shooterRotater, std::shared_ptr<OI> oi) :
  m_shooterRotater {shooterRotater},
  m_OI{oi} {
  lastError = 0,
  isAlignedToHub = false,
  angle = 0_rad,
  targetAngle = 0_rad,
  position = 0_rad,//zeroed position is touching the hard stop
  AddRequirements({m_shooterRotater.get(), m_OI.get()});
}

void RotateTurret::Initialize() {
    std::cerr << "RotateTurret Init" << std::endl;
    Command::Initialize();
  }

// Called repeatedly when this Command is scheduled to run
void RotateTurret::Execute() {
  //TODO: determine direction that robot must be facing in
  m_shooterRotater->SetTargetAngle(targetAngle);

}

// Called once the command ends or is interrupted.
void RotateTurret::End(bool interrupted) {
  if(interrupted) {
        std::cerr << "RotateTurret: Interrupted!" << std::endl;
    }
    Command::End(interrupted);
  }

// Returns true when the command should end.
bool RotateTurret::IsFinished() {
  return false;//TODO: return true if it finishes
}
