// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"
#include <iostream>


using namespace ctre::phoenix6;

SwerveModule::SwerveModule(SwerveModuleConfig cfg, std::shared_ptr<SwerveControlConfig> con_cfg):
    _module_cfg(cfg),
    _control_cfg(con_cfg),
    _steerMotor(_module_cfg.steer_motor_id(), SwerveModuleConfig::canBus), 
    _driveMotor(_module_cfg.drive_motor_id(), SwerveModuleConfig::canBus),
    _steerEncoder(_module_cfg.steer_encoder_id(), SwerveModuleConfig::canBus),
    _driveVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)),
    _steerPositionVoltage(units::angle::turn_t(0.0)),
    _steerVelocitySig{_steerEncoder.GetVelocity(false)}, // Grab the signals we'll need later.
    _steerPositionSig{_steerEncoder.GetPosition(false)},
    _driveVelocitySig{_driveMotor.GetVelocity(false)},
    _drivePositionSig{_driveMotor.GetPosition(false)},
    _driveCurrentSig{_driveMotor.GetTorqueCurrent(false)}
{
    _driveVelocityVoltage.WithSlot(0); // Normal drive control using Slot0.
    _steerPositionVoltage.WithSlot(0); // Normal steering control using Slot0
    
}


bool SwerveModule::configure_hardware() {

    if (!_control_cfg) {
        // TODO: Log configuration errors.
        return false;
    }

    if (!configure_drive_hardware()) {
        // TODO: Log configuration errors.
        return false;
    }

    if (!configure_steer_hardware()) {
        // TODO: Log configuration errors.
        return false;
    }

    return true;
}



const SwerveModule::State& SwerveModule::sample(units::time::microsecond_t now) {

    // Refresh input signals:
    BaseStatusSignal::RefreshAll(_steerPositionSig, _steerVelocitySig, _drivePositionSig, _driveVelocitySig, _driveCurrentSig);

    // Compensate positions:
    auto compensatedSteeringPos = BaseStatusSignal::GetLatencyCompensatedValue(_steerPositionSig, _steerVelocitySig);
    auto compensatedDrivePos = BaseStatusSignal::GetLatencyCompensatedValue(_drivePositionSig, _driveVelocitySig);

    // Now refresh our latest module state based on the latency compensated values:
    _latestState.time_stamp = now;
    // _latestState.drive_velocity = _driveVelocitySig.GetValue(); // TODO: Unit conversions.
    // _latestState.drive_position = _drivePositionSig.GetValue();
    // _latestState.drive_current = _driveCurrentSig.GetValue();

    // _latestState.steering_velocity = _steerVelocitySig.GetValue();
    // _latestState.steering_angle = _steerPositionSig.GetValue();

    // Return our latest state (reference):
    return _latestState;
}


void SwerveModule::set_command(Command cmd) {

    // Convert and send this command to the hardware.

    // Save our latest command.
    _latestCommand = cmd;
}



bool SwerveModule::configure_drive_hardware() {
    configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 30_A;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -30_A;
    

    configs.Voltage.PeakForwardVoltage = 8_V;
    configs.Voltage.PeakReverseVoltage = -8_V;

    // Slot zero for the normal control loop.
    configs.Slot0.kS = 0.0f;
    configs.Slot0.kV = 0.12; // Karken X60:  500kV, 500rpm per V = 8.33 rps per V, 1/8.33 = 0.12 v/rps
    configs.Slot0.kP = 0.11;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kA = 0.0;

    configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

    // Set the control configuration for the drive motor:
    auto status = _driveMotor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        // Log errors.
    }

    // Set our neutral mode to brake on:
    status = _driveMotor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

    if (!status.IsOK()) {
        // Log errors.
    }


    // Drive motors all start out at zero on initialization:
    _driveMotor.SetPosition(units::angle::turn_t(0));

    // Log errors.
    return false;

}

bool SwerveModule::configure_steer_hardware() {

    // Default encoder configuration:
    configs::CANcoderConfiguration encoder_configs{};
    _steerEncoder.GetConfigurator().Apply(encoder_configs, 1_s); 

    // Read back the magnet sensor config for this module.
    configs::MagnetSensorConfigs magSenseConfig;
    auto status = _steerEncoder.GetConfigurator().Refresh(magSenseConfig, 1_s);
    if (!status.IsOK()) {
        // Log error.
    }
    std::cout << "Swerve Module " << _module_cfg.number() << " Encoder Offset: " << magSenseConfig.MagnetOffset.value() << std::endl;

    // Faster signals for steering encoder:
    BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz, _steerPositionSig, _steerVelocitySig);

    // Steering motor configuration:
    configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 30_A;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -30_A;

    configs.Voltage.PeakForwardVoltage = 8_V;
    configs.Voltage.PeakReverseVoltage = -8_V;

    // Slot zero for the normal control loop.
    configs.Slot0.kS = 0.0f;
    configs.Slot0.kV = 0.12; // Karken X60:  500kV, 500rpm per V = 8.33 rps per V, 1/8.33 = 0.12 v/rps
    configs.Slot0.kP = 0.11;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kA = 0.0;

    // Set up steering for using remote CANcoder feedback:
    configs.Feedback.WithFeedbackRemoteSensorID(_module_cfg.steer_encoder_id());
    configs.Feedback.WithFeedbackSensorSource(ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder);
    configs.Feedback.WithRotorToSensorRatio(150.0/7.0);
    configs.Feedback.WithSensorToMechanismRatio(1.0);

    configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive);
    configs.ClosedLoopGeneral.WithContinuousWrap(true);  // Wrapping controls on the steering axis.

    // Set the control configuration for the drive motor:
    status = _driveMotor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        // Log errors.
    }

    // Set our neutral mode to brake on:
    status = _driveMotor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

    if (!status.IsOK()) {
        // Log errors.
    }

    // Log errors.
    return false;
}