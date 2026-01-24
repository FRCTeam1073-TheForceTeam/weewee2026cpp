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
    builder.AddDoubleProperty("StdDev X", [this] { return getStdDevX(); }, [this] (double stdX) { setStdDevX(stdX); });
    builder.AddDoubleProperty("StdDev Y", [this] { return getStdDevY(); }, [this] (double stdY) { setStdDevX(stdY); });
    builder.AddDoubleProperty("StdDev Angle", [this] { return getStdDevA(); }, [this] (double stdA) { setStdDevX(stdA); });
    builder.AddDoubleProperty("Time between updates", [this] { return getTime().value(); },   [this] (double timeGap) { setTime(units::time::millisecond_t(timeGap)); });
    builder.AddDoubleProperty("Linear Speed Thres", [this] { return getLinearSpeed().value(); }, [this] (double linearSpeed) { setTime(units::time::millisecond_t(timeGap));} );
    builder.AddDoubleProperty("Angular Speed Thres", [this] { return getAngularSpeed().value(); }, [this] (double angularSpeed) { setTime(units::time::millisecond_t(timeGap));} );

}

void Localizer::resetPose(frc::Pose2d newPos) {
    frc::SwerveDrivePoseEstimator _estimator(_kinematics, _driveTrain->GetGyroHeadingRadians(), _swerveModulePositions, newPos);
}

void Localizer::Periodic() {
    units::time::second_t now = frc::Timer::GetFPGATimestamp();	
    _estimator.UpdateWithTime(now, _driveTrain->GetGyroHeadingRadians(), _swerveModulePositions);
    if (now - _lastUpdateTime > timeGap && measurementStable()){
        std::vector<AprilTagFinder::VisionMeasurement> measurements = _finder->getAllMeasurements();
        for (int index = 0; index < measurements.size(); index++){
            AprilTagFinder::VisionMeasurement CurrentMeasurement = measurements[index];

            if (CurrentMeasurement._range <= maxRange)
            {
                updateStdDevs(CurrentMeasurement);
                _estimator.AddVisionMeasurement(CurrentMeasurement._pose, CurrentMeasurement._timeStamp, measurementStdDev);
                measurementCounter++;
            }
        }
        _lastUpdateTime = now;
        frc::SmartDashboard::PutNumber("Localize Measurements", measurementCounter);
    }
}

void Localizer::updateStdDevs(AprilTagFinder::VisionMeasurement measurement){
    measurementStdDev[0] = StdDevX + 0.2 * measurement._range.value();
    measurementStdDev[1] = StdDevY + 0.2 * measurement._range.value();
    measurementStdDev[2] = StdDevA + 0.2 * measurement._range.value();
}

bool Localizer::measurementStable(){
        units::meters_per_second_t linearSpeed = units::math::sqrt((_driveTrain->GetChassisSpeeds().vx)*(_driveTrain->GetChassisSpeeds().vx) + (_driveTrain->GetChassisSpeeds().vy)*(_driveTrain->GetChassisSpeeds().vy));
        units::radians_per_second_t angularSpeed = units::math::abs(_driveTrain->GetChassisSpeeds().omega);
        if (linearSpeed <= linearSpeedThreshold && angularSpeed <= angularSpeedThreshold)
        {
            return true;
        }
        else
        {
            return false;
        }
}