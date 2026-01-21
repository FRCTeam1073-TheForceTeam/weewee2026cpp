// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <frc/filter/LinearFilter.h>
// #include <units/length.h>

class IntakePivot : public frc2::SubsystemBase {
 public:

  IntakePivot();
  
  void SetPosition(units::angle::radian_t angle);

  void Periodic() override;

 private:

  units::velocity::meters_per_second_t rollerVelocity;
  units::velocity::meters_per_second_t commandedVelocity;

  // frc::LinearFilter <double> filter;
  // std::span <double, 1> ffGains;
  // std::span <double, 1> fbGains;
  // frc::LinearFilter<double> _LinearFilter(std::span<double>ffGains, std::span<double>fbGains); 
  //ffGains = The "feedforward" or FIR gains. 
  //fbGains = The "Feedback" or IIR gains.

};
