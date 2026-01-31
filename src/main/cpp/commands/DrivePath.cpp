// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DrivePath.h"

DrivePath::DrivePath(std::shared_ptr<Drivetrain> drivetrain, std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) :
  m_drivetrain(drivetrain),
  trajectory(trajectory),
  distanceTolerance(0.1_m),
  angleTolerance(0.1_rad),
  currentSegmentIndex(-1),
  xController{4.8, 0, 0.01},
  yController{4.8, 0, 0.01},
  thetaController{1.5, 0.0, 0.01}
{
  finalOrientation = trajectory.value().GetFinalPose().value().Rotation().Radians();
  thetaController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  frc::SmartDashboard::PutString("Drivepath/Status", "Idle");
  AddRequirements(m_drivetrain.get());
}

// Called when the command is initially scheduled.
void DrivePath::Initialize() {
  startTime = frc::Timer::GetFPGATimestamp();
  currentTime = 0.01_s;
  currentSegmentIndex = 0; //TODO: figure out what to set segment to

  xController.Reset();
  yController.Reset();
  thetaController.Reset();}

// Called repeatedly when this Command is scheduled to run
void DrivePath::Execute() {
  currentTime = frc::Timer::GetFPGATimestamp() - startTime;
  robotPose = m_drivetrain->GetOdometry(); //TODO: put in localizer

  // Path::PathFeedback pathFeedback = path.GetPathFeedback(currentSegmentIndex, robotPose);
  auto sample = trajectory.value().SampleAt(0_s); //TODO: Use current Segment Index
  frc::ChassisSpeeds sample_speed = sample.value().GetChassisSpeeds();
  maxVelocity = 1_mps * std::sqrt(std::pow(sample_speed.vx.value(), 2) + std::sqrt(std::pow(sample_speed.vy.value(), 2))); 
  maxAngularVelocity = std::sqrt(std::pow(sample_speed.vx.value(), 2) + std::sqrt(std::pow(sample_speed.vy.value(), 2))) * 2_rad_per_s;

  if(currentSegmentIndex >= trajectory.value().events.size() - 2) {
    xVelocity = xController.Calculate(robotPose.X().value(), sample_speed.vx.value()) * 1_mps;
    yVelocity = yController.Calculate(robotPose.Y().value(), sample_speed.vy.value()) * 1_mps;
    thetaVelocity = thetaController.Calculate(robotPose.Rotation().Radians().value(), finalOrientation.value()) * 1_rad_per_s;
  }
  else {
    xVelocity = xController.Calculate(robotPose.X().value(), sample_speed.vx.value()) * 1_mps;
    yVelocity = yController.Calculate(robotPose.Y().value(), sample_speed.vy.value()) * 1_mps;
    thetaVelocity = thetaController.Calculate(robotPose.Rotation().Radians().value(), sample.value().GetPose().Rotation().Radians().value()) * 1_rad_per_s;
  }

  xVelocity = std::clamp(xVelocity, -maxVelocity, maxVelocity);
  yVelocity = std::clamp(yVelocity, -maxVelocity, maxVelocity);
  thetaVelocity = units::angular_velocity::radians_per_second_t(std::clamp(thetaVelocity.value(), -maxAngularVelocity.value(), maxAngularVelocity.value()));

  frc::SmartDashboard::PutNumber("DrivePath/TargetX", sample_speed.vx.value());
  frc::SmartDashboard::PutNumber("DrivePath/TargetY", sample_speed.vy.value());
  frc::SmartDashboard::PutNumber("DrivePath/TargetTheta", sample.value().GetPose().Rotation().Radians().value());

  frc::SmartDashboard::PutNumber("DrivePath/MaxVelocity", maxVelocity.value());

  frc::SmartDashboard::PutNumber("DrivePath/CommandedVx", xVelocity.value());
  frc::SmartDashboard::PutNumber("DrivePath/CommandedVy", yVelocity.value());
  frc::SmartDashboard::PutNumber("DrivePath/CommandedVw", thetaVelocity.value());
  frc::SmartDashboard::PutString("DrivePath/SegmentIndex", std::string("Segment Index: %d", currentSegmentIndex));
  frc::SmartDashboard::PutNumber("DrivePath/SegmentSize", trajectory.value().events.size() - 1);
  
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
  if(sample == trajectory.value().GetFinalSample() && currentSegmentIndex + 1 >= trajectory.value().events.size() - 1) { //TODO: need sample to be accessible here (just in execute now)
    frc::SmartDashboard::PutString("DrivePath/Status", "Finished");
    return true;
  }
  return false;
}
