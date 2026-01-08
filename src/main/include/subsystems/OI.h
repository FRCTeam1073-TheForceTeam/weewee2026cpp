#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/force.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <variant>
#include <frc/filter/Debouncer.h>

class OI : public frc2::SubsystemBase {
    public:
    enum class Buttons{
        AButton = 0,
        BButton = 1,
        XButton = 2,
        YButton = 3,
        LeftJoystickY = 1,
        LeftJoystickX = 0,
        LeftJoystickPress = 8,
        RightJoystickY = 5,
        RightJoystickX = 4,
        RightJoystickPress = 9,
        DpadUp = 0,
        DpadLeft = 270,
        DpadRight = 90,
        DpadDown = 180,
        LeftBumper = 4,
        RightBumper = 5,
        LeftTrigger = 2,
        RightTrigger = 3,
        ViewButton = 6,
        MenuButton = 7,
    };
    
    frc::Debouncer m_debouncer{100_ms, frc::Debouncer::DebounceType::kBoth};

    private:
        double LEFT_X_ZERO;
    private:
        double LEFT_Y_ZERO;
    private:    
        double RIGHT_X_ZERO;
    private:
        double RIGHT_Y_ZERO;

};