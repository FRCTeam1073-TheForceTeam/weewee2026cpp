#pragma once

#include "subsystems/Localizer.h"
#include <cmath> 
#include <wpi/SymbolExports.h>
#include <wpi/array.h>
#include <frc/Timer.h>
#include <chrono>

#include "frc/estimator/PoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveDriveOdometry.h"

#include "units/time.h"


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
    /*
    builder.SetSmartDashboardType("Localizer");
    builder.AddDoubleProperty("StdDev X", StdDevX, this::setStdDevX);
    builder.AddDoubleProperty("StdDev Y", this::getStdDevY, this::setStdDevY);
    builder.AddDoubleProperty("StdDev Angle", this::getStdDevA, this::setStdDevA);
    builder.AddDoubleProperty("Time between updates", this::getTime, this::setTime);
    builder.AddDoubleProperty("Linear Speed Thres", this::getLinearSpeed, this::setLinearSpeed);
    builder.AddDoubleProperty("Angular Speed Thres", this::getAngularSpeed, this::setAngularSpeed);
    */

}

void Localizer::resetPose(frc::Pose2d newPos) {
    // estimator = new SwerveDrivePoseEstimator(kinematics, driveTrain.getOdometry().getRotation(), swerveModulePositions, newPos);
    frc::SwerveDrivePoseEstimator _estimator(_kinematics, _driveTrain->GetGyroHeadingRadians(), _swerveModulePositions, newPos);
}

void Localizer::Periodic() {
    units::time::second_t now = frc::Timer::GetFPGATimestamp();	
    //current issue is that updateWithTime is a method of SwerveDrivePoseEstimator in java but isn't in C++. currently working on it
    // _estimator.updateWithTime(now, _driveTrain->GetGyroHeadingRadians(), _swerveModulePositions);

}