// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopDrive.h"
#include <frc/DriverStation.h>

#include <iostream>

TeleopDrive::TeleopDrive(Drivetrain* drivetrain, OI* oi) : m_drivetrain{drivetrain}, m_OI{oi} {
    allianceSign = 1,
    fieldCentric = true,
    lastParkingBreakButton = false,
    lastFieldCentricButton = true,
    parked = false,
    last_error = 0,
    last_snap_time = 0,
    angle_tolerance = 0.05_rad,
    torqueGate = 65_Nm,
    // Register that this command requires the subsystem.
    AddRequirements({drivetrain, oi});
}

void TeleopDrive::Initialize() {
    std::cerr << "TeleopDrive Init" << std::endl;
    Command::Initialize();
    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        allianceSign = -1;
    }
    else {
        allianceSign = -1;
    }
}

void TeleopDrive::Execute() {
    leftY = oi.getDriverLeftY();
    leftX = oi.getDriverLeftX();
    rightX = oi.getDriverRightX();

    frc::SmartDashboard::PutBoolean("TeleopDrive/Parking Break", parked);

    if(oi.GetDriverRightBumper() && lastFieldCentricButton == false) {
        fieldCentric = !fieldCentric;
    }
    lastFieldCentricButton = oi.GetDriverRightBumper();

    if(oi.getDriverLeftBumper() && lastParkingBreakButton == false) {
        parked = !parked;
    }
    lastParkingBreakButton = oi.GetDriverLeftBumper();

    if(parked && !drivetrain.GetParkingBreak()) {
        drivetrain.ParkingBreak(true);
    }
    if(!parked && drivetrain.GetParkingBreak()) {
        drivetrain.ParkingBreak(false);
    }
    else {
        bool dPadUp = oi.GetDriverDPadUp();
        bool dPadDown = oi.GetDriverDPadDown();
        bool dPadLeft = oi.GetDriverDPadLeft();
        bool dPadRight = oi.GetDriverDPadRight();

        if(!(dPadUp || dPadRight || dPadLeft || dPadDown)) {
            mult1 = 1.0 + (oi.GetDriverLeftTrigger() * ((std::sqrt(36)) - 1));
            mult2 = 1.0 + (oi.GetDriverRightTrigger() * ((std::sqrt(36)) - 1));
        }

        //set deadzones
        if(std::abs(leftY) < 0.15) {leftY = 0;}
        if(std::abs(leftX) < 0.15) {leftX = 0;}
        if(std::abs(rightX) < 0.15) {leftX = 0;}

        vx = std::clamp((allianceSign * leftY * maximumLinearVelocity / 25) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        vy = std::clamp((allianceSign * leftX * maximumLinearVelocity / 25) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        w = std::clamp((rightX * maximumRotationVelocity / 25) * mult1 * mult2, -maximumRotationVelocity, maximumRotationVelocity);

        frc::SmartDashboard::PutNumber("TeleopDrive/vx", vx.value());
        frc::SmartDashboard::PutNumber("TeleopDrive/vy", vy.value());
        frc::SmartDashboard::PutNumber("TeleopDrive/w", w.value());
        frc::SmartDashboard::PutBoolean("TeleopDrive/FieldCentric", fieldCentric);

        //TODO: do not know how to set chassis speeds, might not be in code
        Command::Execute();
    }
}

void TeleopDrive::End(bool interuppted) {
    if(interuppted) {
        std::cerr << "TeleopDrive: Interrupted!" << std::endl;
    }
    Command::End(interuppted);
}

bool TeleopDrive::IsFinished() {
   return false;
}


