#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <Eigen/Dense>
#include <vector>

#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Path
    : public frc2::CommandHelper<frc2::Command, Path> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */  
  struct Point {
    Eigen::Vector2d position;
    units::length::meter_t blend_radius = 0.2_m;

    Point() {position << 0.0, 0.0;}

    Point(units::length::meter_t x, units::length::meter_t y) {position << x.value(), y.value();}
  };

  struct Segment {
    Point start;
    Point end;
    Eigen::Vector2d dir;
    units::length::meter_t length;
    units::angle::radian_t orientation;
    units::velocity::meters_per_second_t velocity;
    units::length::meter_t width;

    double orientation_weight;
    double translation_weight;

    Segment(Point start, Point end, units::angle::radian_t orientation, units::velocity::meters_per_second_t velocity);

    void UpdateDirection();
  };

  struct PathFeedback {
    Eigen::Vector2d velocity;
    frc::Pose2d pose;

    PathFeedback(Eigen::Vector2d velocity, frc::Pose2d pose);
  };

  std::vector<Segment> segments;
  units::angle::radian_t finalOrientation;
  units::velocity::meters_per_second_t transverseVelcoity;

  Path(std::vector<Segment> segments, units::angle::radian_t finalOrientation, units::velocity::meters_per_second_t transverseVelcoity);
  
  Eigen::Vector2d PositionToVector(frc::Pose2d pose);

  units::length::meter_t DistanceToSegment(Eigen::Vector2d start, Eigen::Vector2d end, Eigen::Vector2d p, Eigen::Vector2d projection, Eigen::Vector2d direction);

  units::length::meter_t DistanceToSegment(Eigen::Vector2d start, Eigen::Vector2d end, Eigen::Vector2d p);

  units::length::meter_t DistanceToSegment(Eigen::Vector2d start, Eigen::Vector2d end, Eigen::Vector2d p, Eigen::Vector2d projection);

  int ClosestSegment(frc::Pose2d location);

  PathFeedback GetPathFeedback(int segmentIndex, frc::Pose2d location);

  units::angle::radian_t GetPathOrientation(int segmentIndex, frc::Pose2d location);

  bool AtEndPoint(int segmentIndex, frc::Pose2d location);
};

