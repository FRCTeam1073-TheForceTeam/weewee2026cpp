// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/geometry/Translation2d.h>
#include <cmath>

class SwerveModuleConfig {
 public:
  SwerveModuleConfig();

  int moduleNumber;
  const double gearRatio = 6.75;
  const double wheelDiameterMeters = 0.1016;
  frc::Translation2d position;
  const double rotationsPerMeter = gearRatio/(wheelDiameterMeters * 1.06 * M_PI); // 1.06 is the measure correction factor while driving
  const double radiansPerRotation = (150/7) / (2 * M_PI);
  const double steerCurrentLimit = 20;
  const double driveCurrentLimit = 35;
  const double steerCurrentThreshold = 12;
  const double driveCurrentThreshold = 22;
  const double steerCurrentThresholdTime = 0.1;
  const double driveCurrentThresholdTime = 0.25;
  double steerP;
  double steerI;
  double steerD;
  double steerV;
  double driveP;
  double driveI;
  double driveD;
  double driveV;
  double driveA;
  const double driveMaxIntegrator = 400.0;
  const double steerMaxIntegrator = 400.0;


};
