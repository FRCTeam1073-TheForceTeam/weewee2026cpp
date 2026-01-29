// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos/OldTestAuto.h"

std::shared_ptr<Drivetrain> OldTestAuto::m_drivetrain = nullptr;

OldTestAuto::OldTestAuto(std::shared_ptr<Drivetrain> drivetrain)
{
  OldTestAuto::m_drivetrain = drivetrain;
  AddRequirements(m_drivetrain.get());
}

std::unique_ptr<frc2::Command> OldTestAuto::Create() {
  // load the choreo file from deploy folder
  auto trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Test_Auto");


  // follow based on points in path

  auto pathPoses = trajectory.value().GetPoses();

  // TODO: while not have localizer should set odometry to be where first pose is

  // turn the optional Pose2ds into points
  std::vector<Path::Point> pathPoints;
  for(int p = 0; p < pathPoses.size(); p++) {
    pathPoints.push_back(Path::Point(pathPoses[p].X(), pathPoses[p].Y()));
  }

  std::vector<Path::Segment> segments;
  units::angle::radian_t o = 0_rad; // TODO: figure out how to access trajectory orientation
  units::velocity::meters_per_second_t v = 1_mps; // TODO: figure out how to access trajectory velocity
  for(int s = 0; s < pathPoints.size() - 1; s++) {
    segments.push_back(Path::Segment(pathPoints[s], pathPoints[s+1], o, v));
  }

  Path path = Path(segments, o, v);

  return std::make_unique<frc2::SequentialCommandGroup>(DrivePath(OldTestAuto::m_drivetrain, path));

  // units::time::second_t currentTime = timer.GetFPGATimestamp();
  // frc::ChassisSpeeds speeds = trajectory.value().SampleAt(currentTime).value().GetChassisSpeeds();

  // m_drivetrain->SetChassisSpeeds(speeds);
}
