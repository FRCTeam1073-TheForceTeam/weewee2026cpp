// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DrivePath.h"

DrivePath::DrivePath(std::shared_ptr<Drivetrain> drivetrain, Path path) :
  m_drivetrain(drivetrain),
  path(path),
  distanceTolerance(0.1_m),
  angleTolerance(0.1_rad),
  currentSegmentIndex(1),
  xController{4.8, 0, 0.01},
  yController{4.8, 0, 0.01},
  thetaController{1.5, 0.0, 0.01}
{
  thetaController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  frc::SmartDashboard::PutString("Drivepath/Status", "Idle");
  AddRequirements(m_drivetrain.get());
}

// Called when the command is initially scheduled.
void DrivePath::Initialize() {
  startTime = frc::Timer::GetFPGATimestamp();
  currentTime = 0.01_s;
  currentSegmentIndex = 0;

  xController.Reset();
  yController.Reset();
  thetaController.Reset();
  frc::SmartDashboard::PutString("DrivePath/Status", std::string("Starting Segment: %d", currentSegmentIndex));
}

// Called repeatedly when this Command is scheduled to run
void DrivePath::Execute() {
  if(currentSegmentIndex < 0) {
    frc::SmartDashboard::PutString("DrivePath/status", "Invalide segment index.");
    return;
  }

  currentTime = frc::Timer::GetFPGATimestamp() - startTime;
  robotPose = m_drivetrain->GetOdometry();

  Path::PathFeedback pathFeedback = path.GetPathFeedback(currentSegmentIndex, robotPose);
  maxVelocity = units::velocity::meters_per_second_t(pathFeedback.velocity.norm());
  maxAngularVelocity = pathFeedback.velocity.norm() * 2_rad_per_s;

  if(currentSegmentIndex >= path.segments.size() - 1) {
    xVelocity = xController.Calculate(robotPose.X().value(), pathFeedback.pose.X().value()) * 1_mps;
    yVelocity = yController.Calculate(robotPose.Y().value(), pathFeedback.pose.Y().value()) * 1_mps;
    thetaVelocity = thetaController.Calculate(robotPose.Rotation().Radians().value(), path.finalOrientation.value()) * 1_rad_per_s;
  }
  else {
    xVelocity = xController.Calculate(robotPose.X().value(), pathFeedback.pose.X().value()) * 1_mps;
    yVelocity = yController.Calculate(robotPose.Y().value(), pathFeedback.pose.Y().value()) * 1_mps;
    thetaVelocity = thetaController.Calculate(robotPose.Rotation().Radians().value(), path.GetPathOrientation(currentSegmentIndex, robotPose).value()) * 1_rad_per_s;
  }

  xVelocity = std::clamp(xVelocity, -maxVelocity, maxVelocity);
  yVelocity = std::clamp(yVelocity, -maxVelocity, maxVelocity);
  thetaVelocity = units::angular_velocity::radians_per_second_t(std::clamp(thetaVelocity.value(), -maxAngularVelocity.value(), maxAngularVelocity.value()));

  frc::SmartDashboard::PutNumber("DrivePath/TargetX", path.segments[currentSegmentIndex].end.position[0]);
  frc::SmartDashboard::PutNumber("DrivePath/TargetY", path.segments[currentSegmentIndex].end.position[1]);
  frc::SmartDashboard::PutNumber("DrivePath/TargetTheta", path.segments[currentSegmentIndex].orientation.value());

  frc::SmartDashboard::PutNumber("DrivePath/TrajFBVx", pathFeedback.velocity[0]);
  frc::SmartDashboard::PutNumber("DrivePath/TrajFBVy", pathFeedback.velocity[1]);
  frc::SmartDashboard::PutNumber("DrivePath/MaxVelocity", maxVelocity.value());

  frc::SmartDashboard::PutNumber("DrivePath/CommandedVx", xVelocity.value());
  frc::SmartDashboard::PutNumber("DrivePath/CommandedVy", yVelocity.value());
  frc::SmartDashboard::PutNumber("DrivePath/CommandedVw", thetaVelocity.value());
  frc::SmartDashboard::PutString("DrivePath/SegmentIndex", std::string("Segment Index: %d", currentSegmentIndex));
  frc::SmartDashboard::PutNumber("DrivePath/SegmentSize", path.segments.size());
  frc::SmartDashboard::PutNumber("DrivePath/Error", std::sqrt(
    std::pow(path.segments[currentSegmentIndex].end.position[0] - robotPose.X().value(), 2) +
    std::pow(path.segments[currentSegmentIndex].end.position[1] - robotPose.Y().value(), 2)));
  
  m_drivetrain->SetChassisSpeeds(
    frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      xVelocity,
      yVelocity,
      thetaVelocity,
      frc::Rotation2d{m_drivetrain->GetGyroHeadingRadians()} // TODO: replace this angle with localizer one once implemented
    )
  );
}

// Called once the command ends or is interrupted.
void DrivePath::End(bool interrupted) {
  m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s));
}

// Returns true when the command should end.
bool DrivePath::IsFinished() {
  if(path.AtEndPoint(currentSegmentIndex, m_drivetrain->GetOdometry()) && currentSegmentIndex + 1 >= path.segments.size()) {
    frc::SmartDashboard::PutString("DrivePath/Status", "Finished");
    return true;
  }
  return false;
}
