// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopDrive.h"

TeleopDrive::TeleopDrive(Drivetrain* drivetrain)
    : m_drivetrain{drivetrain} {
    // Register that this command requires the subsystem.
    AddRequirements(m_drivetrain);
},

    //TODO: initialize all the stuff
    maximumLinearVelocity = 3.5 distance_m / time_s,
    maximumRotationVelcoity = 3.5 angle_rad / time_s
{}


void TeleopDrive::Initialize() {
}

void TeleopDrive::Execute() {

}

bool TeleopDrive::IsFinished() {

}

void TeleopDrive::End() {

}
