#pragma once

#include "subsystems/DriveTrain.h"
#include "subsystems/AprilTagFinder.h"
#include "subsystems/FieldMap.h"

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Transform3d.h>
#include <frc/Timer.h>
#include <units/time.h>
#include <units/length.h>
#include <cmath> 
#include <wpi/SymbolExports.h>
#include <wpi/array.h>
#include <frc/Timer.h>
#include <chrono>
#include <frc/smartdashboard/SmartDashboard.h>

#include "frc/estimator/PoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveDriveOdometry.h"

#include "units/time.h"

class Localizer : public frc2::SubsystemBase {
    public:

    Localizer(std::shared_ptr<Drivetrain> driveTrain, std::shared_ptr<AprilTagFinder> finder);
    
    void InitSendable(wpi::SendableBuilder &builder) override;

    units::time::millisecond_t getTime() { return timeGap; }

    void setTime(units::time::millisecond_t time) { timeGap = time; }

    // creates an entirely new estimator so the rotation is reset for sure
    void resetPose(frc::Pose2d newPos);

    void resetOrientation();

    units::velocity::meters_per_second_t getLinearSpeed() { return linearSpeedThreshold; }

    void setLinearSpeed(units::velocity::meters_per_second_t speed) { linearSpeedThreshold = speed; }

    units::angular_velocity::radians_per_second_t getAngularSpeed() { return angularSpeedThreshold; }

    void setAngularSpeed(units::angular_velocity::radians_per_second_t angularSpeed) { angularSpeedThreshold = angularSpeed; }

    double getStdDevX() {return StdDevX;}

    void setStdDevX(double devX) {StdDevX = devX;}

    double getStdDevY() {return StdDevY;}

    void setStdDevY(double devY) {StdDevX = devY;}

    double getStdDevA() {return StdDevA;}

    void setStdDevA(double devA) {StdDevA = devA;}

    void Periodic() override;

    frc::Pose2d getPose() { return _estimator->GetEstimatedPosition(); }

    void additionalSensorMeasurement(int id, FieldMap fieldMap);

    bool measurementStable();

    void updateStdDevs(AprilTagFinder::VisionMeasurement measurement);


    private:

    std::shared_ptr<Drivetrain> _driveTrain;    
    std::shared_ptr<AprilTagFinder> _finder;
    std::shared_ptr<frc::SwerveDrivePoseEstimator<4U>> _estimator;

    //apriltag finder here when made
    units::time::second_t _lastUpdateTime;
    wpi::array<double, 3U> measurementStdDev = {0.5,0.5,0.5};
    int measurementCounter = 0;
    units::time::millisecond_t timeGap{8};
    units::velocity::meters_per_second_t linearSpeedThreshold{2.5};
    units::angular_velocity::radians_per_second_t angularSpeedThreshold{2};
    units::meter_t maxRange{3.25};
    double StdDevX = 0.5;
    double StdDevY = 0.5;
    double StdDevA = 0.5;
};