#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>

#include "subsystems/DriveTrain.h"
#include "subsystems/OI.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

class TeleopDrive
    : public frc2::Command {
public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param drivetrain The subsystem used by this command.
   */
    explicit TeleopDrive(Drivetrain* drivetrain, OI* oi);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interupted) override;

private:
    Drivetrain m_drivetrain;
    OI m_OI;

    frc::ChassisSpeeds speeds;

    units::angle::radian_t angle_tolerance;
    frc::Rotation2d rotation;
    frc::PIDController snapPIDProfile{0.05, 0.0, 0.0, 1_s};

    bool fieldCentric;
    bool parked;

    double last_error;
    double last_snap_time;

    bool lastParkingBreakButton;
    bool lastFieldCentricButton;

    static constexpr units::velocity::meters_per_second_t maximumLinearVelocity = 3.5_mps;
    static constexpr units::angular_velocity::radians_per_second_t maximumRotationVelocity = 4.0_rad_per_s;

    double mult1;
    double mult2;

    double leftX;
    double leftY;
    double rightX;

    units::velocity::meters_per_second_t vx;
    units::velocity::meters_per_second_t vy;
    units::angular_velocity::radians_per_second_t w;

    int allianceSign;

    units::torque::newton_meter_t frontLeftTorque;
    units::torque::newton_meter_t frontRightTorque;
    units::torque::newton_meter_t backLeftTorque;
    units::torque::newton_meter_t backRightTorque;
    units::torque::newton_meter_t torqueGate;

    //TODO: aprilTagFinde, Localizer, and Lidar stuff

};
