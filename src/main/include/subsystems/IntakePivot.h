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

    static constexpr int _IntakePivotMotorID1 = 1; //TODO: ID 
    static constexpr int _IntakePivotMotoID2 = 2; //TODO: ID 
    static constexpr int _IntakePivotEncoderID = 3; //TODO: ID

    static constexpr auto TurnsPerMeter = units::angle::turn_t(32.0) / units::length::meter_t(1.0); //TODO: Ratio
    static constexpr auto AmpsPerNewton = units::current::ampere_t(10.0) / units::force::newton_t(1.0); //TODO: Ratio

    static const ctre::phoenix6::CANBus canBus;

    struct Feedback {
        units::length::meter_t position;
        units::velocity::meters_per_second_t velocity;
    };

    IntakePivot();

    const Feedback& GetFeedback() const { return _feedback; }

    void Periodic() override;

    private:

    bool ConfigureHardware();
    bool _hardwareConfigured;

    ctre::phoenix6::hardware::TalonFX _IntakePivotMotor1;
    ctre::phoenix6::hardware::TalonFX _IntakePivotMotor2;
    ctre::phoenix6::hardware::CANcoder _IntakePivotEncoder;

    ctre::phoenix6::StatusSignal<units::angle::turn_t> _IntakePivotPositionSig1;
    ctre::phoenix6::StatusSignal<units::angle::turn_t> _IntakePivotPositionSig2;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _IntakePivotVelocitySig1;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _IntakePivotVelocitySig2;
    ctre::phoenix6::StatusSignal<units::current::ampere_t> _IntakePivotCurrentSig1;
    ctre::phoenix6::StatusSignal<units::current::ampere_t> _IntakePivotCurrentSig2;

    ctre::phoenix6::controls::VelocityVoltage _IntakePivotVelocityVoltage; //Slot 0
    ctre::phoenix6::controls::PositionVoltage _IntakePivotPositionVoltage; //Slot 1

    Feedback _feedback;

};