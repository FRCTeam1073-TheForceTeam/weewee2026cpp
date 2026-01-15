#pragma once

#include "subsystems/DriveTrain.h"

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Transform3d.h>

class Localizer : public frc2::SubsystemBase {


    private:

    Drivetrain _driveTrain;
    frc::SwerveDrivePoseEstimator<4> _estimator;
    //apriltag finder here when made
    units::time::seconds _lastUpdateTime;
    frc::SwerveDriveKinematics<4> _kinematics;
    frc::SwerveModulePosition _swerveModulePositions;
    //std::array<units::angle::radians, 3, 
    //
    int measumentCounter = 0;
    units::time::millisecond_t timeGap{8};
    units::velocity::meters_per_second_t linearSpeedThreshold{2.5};
    units::angle::radian_t angularSpeedThreshold{2};
    units::meter_t maxRange{3.25};



};