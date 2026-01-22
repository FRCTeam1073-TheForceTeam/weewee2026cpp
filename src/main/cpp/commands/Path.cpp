// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Path.h"

Path::Segment::Segment(Point start, Point end, units::radian_t orientation, units::meters_per_second_t velocity) :
orientation(0.0_rad),
velocity(1.0_mps),
orientation_weight(1.0),
translation_weight(1.0),
start(start),
end(end)
{UpdateDirection();}

void Path::Segment::UpdateDirection() {
  length = units::meter_t((end.position - start.position).norm());

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

units::meter_t Path::DistanceToSegment(Eigen::Vector2d begin, Eigen::Vector2d finish, Eigen::Vector2d p, Eigen::Vector2d projection, Eigen::Vector2d direction) { // original
  units::meter_t local_length = units::meter_t((finish - begin).norm());

  if(local_length < 0.001_m) {
    projection << begin[0], begin[1];
    direction << begin[0], begin[1];

    return units::meter_t((p - begin).norm());
  }
  else {
    Eigen::Vector2d dir = (finish - begin) / local_length.value();
    units::meter_t d = units::meter_t((p - begin).dot(dir));

    if(d > local_length) d = local_length;
    if(d < 0.0_m) d = 0.0_m;
    Eigen::Vector2d proj = begin + (dir * d.value());

    projection << proj[0], proj[1];
    Eigen::Vector2d tempD = direction.normalized();
    direction << tempD[0], tempD[1];

    return units::meter_t((p - proj).norm());
  }
}

units::meter_t Path::DistanceToSegment(Eigen::Vector2d begin, Eigen::Vector2d finish, Eigen::Vector2d p) { // without projection or direction
  units::meter_t local_length = units::meter_t((finish - begin).norm());

  if(local_length < 0.001_m) {
    return units::meter_t((p - begin).norm());
  }
  else {
    Eigen::Vector2d dir = (finish - begin) / local_length.value();
    units::meter_t d = units::meter_t((p - begin).dot(dir));

    if(d > local_length) d = local_length;
    if(d < 0.0_m) d = 0.0_m;
    Eigen::Vector2d proj = begin + (dir * d.value());

    return units::meter_t((p - proj).norm());
  }
}

units::meter_t Path::DistanceToSegment(Eigen::Vector2d begin, Eigen::Vector2d finish, Eigen::Vector2d p, Eigen::Vector2d projection) { // without direction
  units::meter_t local_length = units::meter_t((finish - begin).norm());

  if(local_length < 0.001_m) {
    projection << begin[0], begin[1];

    return units::meter_t((p - begin).norm());
  }
  else {
    Eigen::Vector2d dir = (finish - begin) / local_length.value();
    units::meter_t d = units::meter_t((p - begin).dot(dir));

    if(d > local_length) d = local_length;
    if(d < 0.0_m) d = 0.0_m;
    Eigen::Vector2d proj = begin + (dir * d.value());

    projection << proj[0], proj[1];

    return units::meter_t((p - proj).norm());
  }
}

int Path::ClosestSegment(frc::Pose2d location) {
  Eigen::Vector2d pos = PositionToVector(location);
  int closestIndex = -1;
  units::meter_t closestDistance = 9999.9_m;

  for(int segmentIndex = 0; segmentIndex < segments.size(); segmentIndex++) {
    Segment seg = segments[segmentIndex];

    //TODO: use a pointer and fix the falsey check
    units::meter_t segDist = DistanceToSegment(seg.start.position, seg.end.position, pos);
    if (segDist < closestDistance) {
      closestIndex = segmentIndex;
      closestDistance = segDist;
    }
  }
  return closestIndex;
}

Path::PathFeedback Path::GetPathFeedback(int segmentIndex, frc::Pose2d location) {
  Eigen::Vector2d Vp;
  Eigen::Vector2d pos = PositionToVector(location);
  Eigen::Vector2d path_pos;
  Eigen::Vector2d path_dir;

  double proportion;

  Segment seg = segments[segmentIndex];
  units::meter_t path_offset = DistanceToSegment(seg.start.position, seg.end.position, pos, path_pos, path_dir);

  frc::SmartDashboard::PutNumber("Path/path_offset", path_offset.value());

  if(path_offset < seg.width) {
    proportion = path_offset / seg.width;
    Vp = path_dir * (1.0 - proportion) * seg.velocity.value(); // TODO: probably should do better unit conversion
  }
  else {
    proportion = 0.0;
    Vp.fill(0);
  }
  frc::SmartDashboard::PutNumber("Path/proportion", proportion);

  Eigen::Vector2d pp = path_pos + (Vp * 0.2);
  Eigen::Vector2d ppp;

  DistanceToSegment(seg.start.position, seg.end.position, pp, ppp); // TODO figure out and fix distance to segment overloaded methods
  frc::Pose2d pathPoint = frc::Pose2d(units::meter_t(ppp[0]), units::meter_t(ppp[1]), frc::Rotation2d(seg.orientation));
  
  frc::SmartDashboard::PutNumber("Path/ProjectionX", ppp[0]);
  frc::SmartDashboard::PutNumber("Path/ProjectionY", ppp[1]);

  return Path::PathFeedback(Vp, pathPoint);
}

units::radian_t Path::GetPathOrientation(int segmentIndex, frc::Pose2d location) {
  return segments[segmentIndex].orientation;
}

bool Path::AtEndPoint(int segmentIndex, frc::Pose2d location) {
  Eigen::Vector2d pos = PositionToVector(location);
  units::meter_t dist = units::meter_t((segments[segmentIndex].end.position - pos).norm());

  if(dist < segments[segmentIndex].end.blend_radius) {
    return true;
  }
  return false;
}
