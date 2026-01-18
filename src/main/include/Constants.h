// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/angular_velocity.h>
#include <units/voltage.h>


/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace Constants {

class Operator {
    public:
    static constexpr int kDriverControllerPort = 0;
};


class Drivetrain {
    public:
    static constexpr float foo = 0.0f;

};

// Special backwards unit Phoenix6 / CTRE Library uses.
using VoltsPerRPS =  units::compound_unit<units::voltage::volt, units::inverse<units::angular_velocity::turns_per_second>>;

class KrakenX60 {
    public:
    static constexpr auto kV = 509.3_rpm / 1.0_V;
};

class KrakenX44 {
    public:
    static constexpr auto kV = 653.8_rpm / 1.0_V;
};

class Falcon {
    public:
    static constexpr auto kV = 534.8_rpm / 1.0_V;
};


}  // namespace Constants
