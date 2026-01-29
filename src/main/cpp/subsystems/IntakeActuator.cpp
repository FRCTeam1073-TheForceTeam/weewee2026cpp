// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeActuator.h"
#include <iostream>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6;
using namespace units;


IntakeActuator::IntakeActuator(): 
    _hardwareConfigured(true),
    _LeadMotor(_LeadMotorID, CANBus("rio")), 
    _FollowMotor(_FollowMotorID, CANBus("rio")), 
    _Encoder(_EncoderID, CANBus("rio")),
    _PositionSig(_LeadMotor.GetPosition()),
    _VelocitySig(_LeadMotor.GetVelocity()),
    _CurrentSig(_LeadMotor.GetTorqueCurrent()),
    _VelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)), //TODO: Get Velocity
    _PositionVoltage(units::angle::turn_t(0.0)) //TODO: Get Velocity
{
    _VelocityVoltage.WithSlot(0);
    _PositionVoltage.WithSlot(1);

    _hardwareConfigured = ConfigureHardware();
    if (!_hardwareConfigured) {
        std::cerr << "IntakeActuator: Hardware Failed To Configure!" << std::endl;
    }

    _command = std::monostate();

} 

void IntakeActuator::SetIntakeVelocity(units::angular_velocity::turns_per_second_t Velocity) {
  _TargetVelocity = Velocity;
}

ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> IntakeActuator::GetIntakeVelocity() {
  return _VelocitySig;
}

units::angular_velocity::turns_per_second_t IntakeActuator::GetIntakeTargetVelocity() {
  return _TargetVelocity;
}

void IntakeActuator::Periodic() {
    
    BaseStatusSignal::RefreshAll(_PositionSig, _VelocitySig, _CurrentSig);

    _FollowMotor.SetControl(controls::StrictFollower{_LeadMotor.GetDeviceID()});

    if (_voltageSignal.GetValue() > volt_t(5)) { //TODO: Get Value
      // _LeadMotor.SetPosition(units::angle::turn_t(0)),
      _LeadMotor.SetVoltage(volt_t(0));
      _LeadMotor.StopMotor();
    } 

}

bool IntakeActuator::ConfigureHardware() {
    configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; //TODO: Get Peak
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; //TODO: Get Peak

    configs.Voltage.PeakForwardVoltage = 8_V; //TODO: Get Peak
    configs.Voltage.PeakReverseVoltage = -8_V; //TODO: Geat Peak

    // Slot 0, TODO: Get Numbers
    configs.Slot0.kV = 0.12;
    configs.Slot0.kP = 0.35;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.03;
    configs.Slot0.kA = 0.0;

    // Slot 1, TODO: Get Numbers
    configs.Slot1.kV = 0.12;
    configs.Slot1.kP = 0.1;
    configs.Slot1.kI = 0.01;
    configs.Slot1.kD = 0.0;
    configs.Slot1.kA = 0.0;

    auto status = _LeadMotor.GetConfigurator().Apply(configs, 1_s);

    configs::TalonFXConfiguration followerConfigs{};
    followerConfigs.MotorOutput.WithInverted(signals::InvertedValue::CounterClockwise_Positive);

    _LeadMotor.SetVoltage(volt_t(0));
    
    if (!status.IsOK()) {
        std::cerr << "IntakeActuator is not working" << std::endl;
    }

    return true;

}