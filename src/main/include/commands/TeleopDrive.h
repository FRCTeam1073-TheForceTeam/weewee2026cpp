#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrain.h"
#include "subsystems/OI.h"
#include <frc/controller/ProfiledPIDController.h>

class TeleopDrive
    : public frc2::Command {
public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param drivetrain The subsystem used by this command.
   */
    explicit TeleopDrive(Drivetrain* drivetrain);

    void Initialize();
    void Execute();
    bool IsFinished();
    void End();

private:

    Drivetrain drivetrain;
    OI m_OI;
    Drivetrain* m_drivetrain;

    frc::ChassisSpeeds speeds;

    units::radians angle_tolerance;
    frc::Rotation2d rotation;
    // frc::PIDController snapPIDProfile;

    bool fieldCentric;
    bool parked;

    double last_error;
    double last_snap_time;

    bool lastParkingBreakButton;
    bool lastFieldCentricButton;

    static const units::velocity::meters_per_second maximumLinearVelocity;
    static const units::rad_per_s maximumRotationVelocity;

    double mult1;
    double mult2;

    double leftX;
    double leftY;
    double rightX;

    units::velocity::meters_per_second vx;
    units::velocity::meters_per_second vy;
    units::rad_per_s w;

    int allianceSign;

    units::torque::meter_kilogram frontLeftTorque;
    units::torque::meter_kilogram frontRightTorque;
    units::torque::meter_kilogram backLeftTorque;
    units::torque::meter_kilogram backRightTorque;
    units::torque::meter_kilogram torqueGate;

    //TODO: aprilTagFinde, Localizer, and Lidar stuff

};
