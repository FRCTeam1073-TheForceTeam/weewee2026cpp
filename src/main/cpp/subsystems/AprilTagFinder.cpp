#include "subsystems/AprilTagFinder.h"

std::vector<AprilTagFinder::RobotCamera> AprilTagFinder::_cameras = {
};

frc::Pose3d AprilTagFinder::estimateFieldToRobotAprilTag(frc::Transform3d cameraToTarget, frc::Pose3d fieldRelativeTagPose, frc::Transform3d cameraToRobot) {
    return fieldRelativeTagPose.TransformBy(cameraToTarget.Inverse()).TransformBy(cameraToRobot);
}

AprilTagFinder::VisionMeasurement::VisionMeasurement(frc::Pose2d pose, frc::Transform2d relativePose, units::second_t timeStamp, int tagID, units::meter_t range) :
    _pose(pose),
    _relativePose(relativePose),
    _timeStamp(timeStamp),
    _tagID(tagID),
    _range(range)
{}

std::vector<AprilTagFinder::VisionMeasurement> AprilTagFinder::getAllMeasurements() {
    return _visionMeasurements;
}

std::vector<photon::PhotonTrackedTarget> AprilTagFinder::getCamTargets(photon::PhotonCamera& camera) {
    std::vector<photon::PhotonPipelineResult> results = camera.GetAllUnreadResults();
    std::vector<photon::PhotonTrackedTarget> targets = std::vector<photon::PhotonTrackedTarget>();

    for(auto& result : results){
        if(result.HasTargets()){
            //I don't know a good way of turning a std::span into an std::vector so I just did this
            std::span<const photon::PhotonTrackedTarget> r = result.GetTargets();
            for(auto& c : r)
            {
                targets.push_back(c);
            }
        }
    }
    return targets;
}

std::vector<AprilTagFinder::VisionMeasurement> AprilTagFinder::getCamMeasurements(photon::PhotonCamera& camera, frc::Transform3d camTransform3d) {
    std::vector<VisionMeasurement> measurements = std::vector<VisionMeasurement>();
    std::vector<photon::PhotonPipelineResult> results = camera.GetAllUnreadResults();

    for (auto& result : results){
        if (result.HasTargets()){
          //targets.addAll(result.getTargets());
        
        units::time::second_t responseTimestamp = frc::Timer::GetFPGATimestamp();//- result.metadata.getLatencyMillis() / 1000.0;
        units::meter_t range = 0_m;
        for (auto& target : result.GetTargets()) {
            if (FieldMap::fieldMap.GetTagPose(target.GetFiducialId()).has_value()){
                if (target.GetPoseAmbiguity() != -1 && target.GetPoseAmbiguity() < ambiguityThreshold){
                frc::Transform3d best = target.GetBestCameraToTarget();
                frc::Pose3d robotPose = estimateFieldToRobotAprilTag(best,
                    FieldMap::fieldMap.GetTagPose(target.GetFiducialId()).value(), 
                    camTransform3d.Inverse());
                range = target.bestCameraToTarget.Translation().Norm();
                frc::Transform2d relativePose = toTransform2d(camTransform3d+best);
                measurements.push_back(VisionMeasurement(robotPose.ToPose2d(), relativePose, responseTimestamp, target.GetFiducialId(), range));
                }
            }
        }
      }
    }
    return measurements;
}

frc::Transform2d AprilTagFinder::toTransform2d(frc::Transform3d t3d) {
    return frc::Transform2d(t3d.X(), t3d.Y(), t3d.Rotation().ToRotation2d());
}

frc::Transform3d AprilTagFinder::getRobotCam(int index) {
    return _cameras[index].transform;
}

void AprilTagFinder::Periodic() {
    _visionMeasurements.clear();
    for (auto& cam : _cameras) {
        std::vector<AprilTagFinder::VisionMeasurement> measurements = getCamMeasurements(cam.camera, cam.transform);
        _visionMeasurements.insert(
            _visionMeasurements.end(),
            measurements.begin(),
            measurements.end()
        );
    }
}