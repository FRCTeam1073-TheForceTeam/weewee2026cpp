#pragma once

#include "subsystems/Localizer.h"

Localizer::Localizer(std::shared_ptr<Drivetrain> driveTrain, std::shared_ptr<FieldMap> fieldMap, std::shared_ptr<AprilTagFinder> finder) : 
    _driveTrain(driveTrain),
    _fieldMap(fieldMap),
    _finder(finder),
    _kinematics(driveTrain->GetKinematics()),
    _swerveModulePositions(driveTrain->GetSwerveModulePositions()),
    _estimator(frc::SwerveDrivePoseEstimator(_kinematics, driveTrain->GetOdometry().Rotation(), _swerveModulePositions, frc::Pose2d())),
    _lastUpdateTime(frc::Timer::GetFPGATimestamp())
{};

void Localizer::InitSendable(wpi::SendableBuilder &builder) {
    builder.SetSmartDashboardType("Localizer");
    /*builder.AddDoubleProperty("StdDev X", StdDevX, this::setStdDevX);
    builder.AddDoubleProperty("StdDev Y", this::getStdDevY, this::setStdDevY);
    builder.AddDoubleProperty("StdDev Angle", this::getStdDevA, this::setStdDevA);
    builder.AddDoubleProperty("Time between updates", this::getTime, this::setTime);
    builder.AddDoubleProperty("Linear Speed Thres", this::getLinearSpeed, this::setLinearSpeed);
    builder.AddDoubleProperty("Angular Speed Thres", this::getAngularSpeed, this::setAngularSpeed);
    */
}

void Localizer::resetPose(frc::Pose2d newPos) {
    
}

void Localizer::Periodic() {
    
}