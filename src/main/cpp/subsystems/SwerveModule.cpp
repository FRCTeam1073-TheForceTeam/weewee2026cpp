// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"
#include <iostream>

using namespace ctre::phoenix6;

SwerveModule::SwerveModule(Ids ids, frc::Translation2d location, const std::string& canBus) :
    _ids(ids), 
    _location(std::move(location)), 
    _hardwareConfigured(true),
    _steerMotor(ids.steerMotorId, canBus), 
    _driveMotor(ids.driveMotorId, canBus),
    _steerEncoder(ids.steerEncoderId, canBus),
    _driveVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)),
    _steerPositionVoltage(units::angle::turn_t(0.0)),
    _steerVelocitySig{_steerEncoder.GetVelocity(false)}, // Grab the signals we'll need later.
    _steerPositionSig{_steerEncoder.GetPosition(false)},
    _driveVelocitySig{_driveMotor.GetVelocity(false)},
    _drivePositionSig{_driveMotor.GetPosition(false)},
    _driveCurrentSig{_driveMotor.GetTorqueCurrent(false)}
{
    // Start out not moving:
    _targetState.angle = 0_deg;
    _targetState.speed = 0_mps;

    // Controllers use slot 0:
    _driveVelocityVoltage.WithSlot(0); // Normal drive control using Slot0.
    _steerPositionVoltage.WithSlot(0); // Normal steering control using Slot0
    
    // Try to configure the hardware once we're constructed and remember if we succeed.
    _hardwareConfigured &= ConfigureHardware();

    if (!_hardwareConfigured) {
        std::cerr << "SwerveModule[" << _ids.number << "] failed to configure!" << std::endl;
    } else {
        std::cerr << "SwerveModule[" << _ids.number << "] configured." << std::endl;
    }
}

bool SwerveModule::IsConfigurationValid() const {
    if (_ids.number < 0) return false;
    if (_ids.driveMotorId < 0) return false;
    if (_ids.steerMotorId < 0) return false;
    if (_ids.steerEncoderId < 0) return false;

    return true;
}


bool SwerveModule::ConfigureHardware() {
    bool configured = true;

    if (!IsConfigurationValid()) {
        std::cerr << "SwerveModule[" << _ids.number << "] failed to configure drive hardware!" << std::endl;
        configured = false;
    }
    if (!ConfigureDriveHardware()) {
        std::cerr << "SwerveModule[" << _ids.number << "] failed to configure drive hardware!" << std::endl;
        configured = false;
    }

    if (!ConfigureSteerHardware()) {
        std::cerr << "SwerveModule[" << _ids.number << "] failed to configure steer hardware!" << std::endl;
        configured = false;
    }



    // Sample the hardware state once we're configured.
    SampleState(frc::Timer::GetFPGATimestamp());

    return configured;
}



const SwerveModule::DetailedState& SwerveModule::SampleState(units::time::second_t now) {

    // Refresh input signals:
    BaseStatusSignal::RefreshAll(_steerPositionSig, _steerVelocitySig, _drivePositionSig, _driveVelocitySig, _driveCurrentSig);

    // Latency compensated positions:
    auto compensatedSteeringPos = BaseStatusSignal::GetLatencyCompensatedValue(_steerPositionSig, _steerVelocitySig);
    auto compensatedDrivePos = BaseStatusSignal::GetLatencyCompensatedValue(_drivePositionSig, _driveVelocitySig);

    // Now refresh our latest module state based on the latency compensated values:
    _latestState.timeStamp = now;
    _latestState.driveVelocity = _driveVelocitySig.GetValue() * SwerveControlConfig::DriveMetersPerMotorTurn;
    _latestState.drivePosition = _drivePositionSig.GetValue() * SwerveControlConfig::DriveMetersPerMotorTurn;
    _latestState.driveCurrent = _driveCurrentSig.GetValue();

    // Steering Axis incorporates the gear ratio in the control setup:
    _latestState.steeringVelocity = _steerVelocitySig.GetValue();
    _latestState.steeringAngle = _steerPositionSig.GetValue();


    // Compute two simpler outputs:

    // Swerve module state:
    _latestSwerveModuleState.angle = _latestState.steeringAngle;
    _latestSwerveModuleState.speed = _latestState.driveVelocity;

    // Swerve module position:
    _latestSwerveModulePosition.angle = _latestState.steeringAngle;
    _latestSwerveModulePosition.distance = _latestState.drivePosition;

    // Return our latest state (reference):
    return _latestState;
}


void SwerveModule::SetCommand(frc::SwerveModuleState cmd) {

    if (!_hardwareConfigured) return; // No controls if configure failed.

    // Cache command for reference later.
    _targetState = cmd;

    // Convert and send this command to the hardware.
    auto drive_motor_velocity = _targetState.speed / SwerveControlConfig::DriveMetersPerMotorTurn;
    auto steering_angle = units::angle::degree_t(_targetState.angle.Degrees());

    // Controller commands.
    _driveMotor.SetControl(_driveVelocityVoltage.WithVelocity(drive_motor_velocity));
    _steerMotor.SetControl(_steerPositionVoltage.WithPosition(steering_angle));
}

  void SwerveModule::SetDriveBrakeMode(bool brake) {
    if (brake)
        _driveMotor.SetNeutralMode(signals::NeutralModeValue::Brake);
    else
        _driveMotor.SetNeutralMode(signals::NeutralModeValue::Coast);
  }

bool SwerveModule::ConfigureDriveHardware() {
    configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = SwerveControlConfig::DriveCurrentLimit;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveControlConfig::DriveCurrentLimit;
    

    configs.Voltage.PeakForwardVoltage = 8_V;
    configs.Voltage.PeakReverseVoltage = -8_V;

    // Slot zero for the normal control loop:
    configs.Slot0 = SwerveControlConfig::GetDriveControlConfig();


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

bool SwerveModule::ConfigureSteerHardware() {

    // Default encoder configuration:
    configs::CANcoderConfiguration  encoder_configs;
    _steerEncoder.GetConfigurator().Apply(encoder_configs, 1_s); 

    // Read back the magnet sensor config for this module.
    configs::MagnetSensorConfigs magSenseConfig;
    auto status = _steerEncoder.GetConfigurator().Refresh(magSenseConfig, 1_s);
    if (!status.IsOK()) {
        // Log error.
    }

    std::cout << "SwerveModule [" << _ids.number << "] Encoder offset: " << magSenseConfig.MagnetOffset.value() << std::endl;

    // Faster signals for steering encoder:
    BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz, _steerPositionSig, _steerVelocitySig);

    // Steering motor configuration:
    configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = SwerveControlConfig::SteerCurrentLimit;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveControlConfig::SteerCurrentLimit;

    configs.Voltage.PeakForwardVoltage = 8_V;
    configs.Voltage.PeakReverseVoltage = -8_V;

    // Slot zero for the normal control loop.
    configs.Slot0 = SwerveControlConfig::GetSteerControlConfig();

    // Set up steering for using remote CANcoder feedback:
    configs.Feedback.WithFeedbackRemoteSensorID(_ids.steerEncoderId);
    configs.Feedback.WithFeedbackSensorSource(ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder);
    configs.Feedback.WithRotorToSensorRatio(SwerveControlConfig::SteerGearRatio.value());
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