// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "subsystems/FieldMap.h"
#include <frc2/command/SubsystemBase.h>
#include <vector>

//Photon Vision Includes
#include <photon/PhotonCamera.h>
#include <photon/PhotonUtils.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/Timer.h>
#include <units/time.h>
#include <units/length.h>
#include <units/angle.h>


class AprilTagFinder : public frc2::SubsystemBase {
    public:
    AprilTagFinder();
    class VisionMeasurement
    {
    public:
        frc::Pose2d _pose; // this is in field coordinates
        frc::Transform2d _relativePose; // this is in robot coordinates.
        units::second_t _timeStamp; //this might not be in seconds I'm not sure yet
        int _tagID;
        units::meter_t _range;
        VisionMeasurement(frc::Pose2d pose, frc::Transform2d relativePose, units::second_t timeStamp, int tagID, units::meter_t range);
    };
    struct RobotCamera{
    public: 
        std::shared_ptr<photon::PhotonCamera> _camera;
        frc::Transform3d _transform;
        RobotCamera(std::shared_ptr<photon::PhotonCamera> camera, frc::Transform3d transform) : _camera(camera), _transform(transform) {};
    };
    
    static frc::Pose3d estimateFieldToRobotAprilTag(frc::Transform3d cameraToTarget, frc::Pose3d fieldRelativeTagPose, frc::Transform3d cameraToRobot);

    std::vector<VisionMeasurement> getAllMeasurements();

    std::vector<photon::PhotonTrackedTarget> getCamTargets(std::shared_ptr<photon::PhotonCamera> camera);

    frc::Transform2d toTransform2d(frc::Transform3d t3d);

    std::vector<VisionMeasurement> getCamMeasurements(std::shared_ptr<photon::PhotonCamera> camera, frc::Transform3d camTransform3d);
    
    frc::Transform3d getRobotCam(int index);

    void Periodic() override;
    
    static std::vector<RobotCamera> _cameras;
    
    const double ambiguityThreshold = 0.28;
    private:
        std::vector<VisionMeasurement> _visionMeasurements;

};