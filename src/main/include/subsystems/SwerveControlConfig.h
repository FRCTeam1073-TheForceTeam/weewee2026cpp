
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/math.h>
#include <units/constants.h>
#include <cmath>



/**
 * This class is meant to hold the shared, common control configuration for the swerve drive modules.
 * There is only one shared instance of this class in a robot and all swerve modules share the same
 * controller configurations using a shared_ptr<> to it.
 */
class SwerveControlConfig {
    public:

    SwerveControlConfig();

    // static const units::length::meter_t wheel_diameter = units::length::meter_t(0.1016);
    // static const units::dimensionless::scalar_t gear_ratio = units::dimensionless::scalar_t(6.75);
    // static const auto rotations_per_meter = gear_ratio/(units::constants::PI() * wheel_diameter);

    // Drive Constants:
    static constexpr units::length::meter_t drive_wheel_diameter = 0.1016_m;
    static constexpr units::dimensionless::scalar_t drive_gear_ratio = 6.75;
    static constexpr units::unit_t<units::detail::unit_multiply<units::angle::turns, units::inverse<units::length::meter>>, double, units::linear_scale> drive_rotations_per_meter =
        units::angle::turn_t(1) * drive_gear_ratio / (drive_wheel_diameter * units::constants::pi);
    static constexpr units::current::ampere_t drive_current_limit = 35.0_A;

    // Steer Constants:
    static constexpr units::dimensionless::scalar_t steer_gear_ratio = (150.0/7.0);
    static constexpr units::current::ampere_t steer_current_limit = 20.0_A;

    // Controller Configurations:
    ctre::phoenix6::configs::SlotConfigs drive_control_config() const;
    ctre::phoenix6::configs::SlotConfigs steer_control_config() const;


    private:

    // const double gearRatio = 6.75f;
    // const double wheelDiameterMeters = 0.1016f;
    // const double rotationsPerMeter = gearRatio/(wheelDiameterMeters * 1.06 * M_PI); // 1.06 is the measure correction factor while driving


    // const double radiansPerRotation = (150/7) / (2.0 * M_PI);
    // const double steerCurrentLimit = 20;
    // const double driveCurrentLimit = 35;
    // const double steerCurrentThreshold = 12;
    // const double driveCurrentThreshold = 22;
    // const double steerCurrentThresholdTime = 0.1;
    // const double driveCurrentThresholdTime = 0.25;
    // double steerP;
    // double steerI;
    // double steerD;
    // double steerV;
    // double driveP;
    // double driveI;
    // double driveD;
    // double driveV;
    // double driveA;
    // const double driveMaxIntegrator = 400.0;
    // const double steerMaxIntegrator = 400.0;

};
