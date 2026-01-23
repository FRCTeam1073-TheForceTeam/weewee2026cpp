// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakePivot.h"
#include <iostream>

using namespace ctre::phoenix6;

IntakePivot::IntakePivot(): 
    _hardwareConfigured(true),
    _IntakePivotMotor1(_IntakePivotMotorID1, CANBus("rio")), 
    _IntakePivotMotor2(_IntakePivotMotorID1, CANBus("rio")), 
    _IntakePivotEncoder(_IntakePivotEncoderID, CANBus("rio")),
    _IntakePivotPositionSig1(_IntakePivotMotor1.GetPosition()),
    _IntakePivotPositionSig2(_IntakePivotMotor2.GetPosition()),
    _IntakePivotVelocitySig1(_IntakePivotMotor1.GetVelocity()),
    _IntakePivotVelocitySig2(_IntakePivotMotor1.GetVelocity()),
    _IntakePivotCurrentSig1(_IntakePivotMotor1.GetTorqueCurrent()),
    _IntakePivotCurrentSig2(_IntakePivotMotor1.GetTorqueCurrent()),
    _IntakePivotVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)), //TODO: Get Velocity
    _IntakePivotPositionVoltage(units::angle::turn_t(0.0)) //TODO: Get Velocity
{
    _IntakePivotVelocityVoltage.WithSlot(0);
    _IntakePivotPositionVoltage.WithSlot(1);

    _hardwareConfigured = ConfigureHardware();
    if (!_hardwareConfigured) {
        std::cerr << "IntakePivot: Hardware Failed To Configure! Yell At Cole!" << std::endl;
    }

} 

void IntakePivot::Periodic() {
    
    BaseStatusSignal::RefreshAll(_IntakePivotPositionSig1, _IntakePivotPositionSig2, _IntakePivotVelocitySig1, _IntakePivotVelocitySig2, _IntakePivotCurrentSig1, _IntakePivotCurrentSig2);

}

bool IntakePivot::ConfigureHardware() {
    configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; //TODO: Get Peak
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; //TODO: Get Peak

    configs.Voltage.PeakForwardVoltage = 8_V; //TODO: Get Peak
    configs.Voltage.PeakReverseVoltage = -8_V; //TODO: Geat Peak

    //TODO: Add Slot 0

    //TODO: Add Slot 1

}