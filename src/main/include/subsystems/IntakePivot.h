// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <wpi/sendable/SendableBuilder.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/force.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>

#include <variant>

class IntakePivot : public frc2::SubsystemBase {
    public:

    static constexpr auto TurnsPerMeter = units::angle::turn_t(32.0) / units::length::meter_t(1.0);
    static constexpr auto AmpsPerNewton = units::current::ampere_t(10.0) / units::force::newton_t(1.0);

    static const ctre::phoenix6::CANBus canBus;

    IntakePivot();
    

    //void Periodic() override;

    private:

    bool ConfigureHardware();
    bool _hardwareConfigured;

    ctre::phoenix6::hardware::TalonFX _IntakePivotMotor;
    ctre::phoenix6::hardware::CANcoder _IntakePivotEncoder;

};