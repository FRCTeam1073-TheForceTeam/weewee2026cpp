// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Path.h"

Path::Segment::Segment(Point start, Point end, units::angle::radian_t orientation, units::velocity::meters_per_second_t velocity) :
orientation(0.0_rad),
velocity(1.0_mps),
orientation_weight(1.0),
translation_weight(1.0),
start(start),
end(end)
{UpdateDirection();}

void Path::Segment::UpdateDirection() {
  length = units::length::meter_t((end.position - start.position).norm());

  if(length > 0.001_m) {
    dir = (end.position - start.position) / length.value();
  }
  else {
    dir << 0.0, 0.0;
  }
}

Path::PathFeedback::PathFeedback(Eigen::Vector2d velocity, frc::Pose2d pose) :
  velocity(velocity),
  pose(pose)
{}

Eigen::Vector2d Path::PositionToVector(frc::Pose2d pose) {
  Eigen::Vector2d v = {pose.X(), pose.Y()};
  return v;
}

units::length::meter_t Path::DistanceToSegment(Eigen::Vector2d begin, Eigen::Vector2d finish, Eigen::Vector2d p, Eigen::Vector2d projection, Eigen::Vector2d direction) {
  units::length::meter_t local_length = units::length::meter_t((finish - begin).norm());

  if(local_length < 0.001_m) {
    projection << begin[0], begin[1];
    direction << begin[0], begin[1];

    return units::length::meter_t((p - begin).norm());
  }
  else {
    Eigen::Vector2d direction = (finish - begin) / local_length.value();
    units::length::meter_t d = units::length::meter_t((p - begin).dot(direction));

    if(d > local_length) d = local_length;
    if(d < 0.0_m) d = 0.0_m;
    Eigen::Vector2d proj = begin + (direction * d.value());

    projection << proj[0], proj[1];
    Eigen::Vector2d tempD = direction.normalized();
    direction << tempD[0], tempD[1];

    return units::length::meter_t((p - proj).norm());
  }
}

units::length::meter_t Path::DistanceToSegment(Eigen::Vector2d begin, Eigen::Vector2d finish, Eigen::Vector2d p) {
  units::length::meter_t local_length = units::length::meter_t((finish - begin).norm());

  if(local_length < 0.001_m) {
    return units::length::meter_t((p - begin).norm());
  }
  else {
    Eigen::Vector2d direction = (finish - begin) / local_length.value();
    units::length::meter_t d = units::length::meter_t((p - begin).dot(direction));

    if(d > local_length) d = local_length;
    if(d < 0.0_m) d = 0.0_m;
    Eigen::Vector2d proj = begin + (direction * d.value());
    
    return units::length::meter_t((p - proj).norm());
  }
}

int Path::ClosestSegment(frc::Pose2d location) {
  Eigen::Vector2d pos = PositionToVector(location);
  int closestIndex = -1;
  units::length::meter_t closestDistance = 9999.9_m;

  for(int segmentIndex = 0; segmentIndex < segments.size(); segmentIndex++) {
    Segment seg = segments[segmentIndex];

    //TODO: use a pointer and fix the falsey check
    units::length::meter_t segDist = DistanceToSegment(seg.start.position, seg.end.position, pos);
    if (segDist < closestDistance) {
      closestIndex = segmentIndex;
      closestDistance = segDist;
    }
  }
  return closestIndex;
}



// Called when the command is initially scheduled.
void Path::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Path::Execute() {}

// Called once the command ends or is interrupted.
void Path::End(bool interrupted) {}

// Returns true when the command should end.
bool Path::IsFinished() {
  return false;
}

