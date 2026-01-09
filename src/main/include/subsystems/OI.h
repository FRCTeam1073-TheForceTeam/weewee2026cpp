#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/force.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <variant>
#include <frc/filter/Debouncer.h>
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

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
        
        // feedback
        struct Feedback {
            units::radians leftStickX; //TODO: put in the other values
        };

        // Constructor
        OI();

        void Periodic() override;
        
        // access the feedback from the controllers
        const Feedback& GetFeedback() const { return _feedback; }


        //TODO: figure out what these units actually should be
        units::radians GetDriverLeftX();
        units::radians GetDriverLeftY();
        units::radians GetDriverRightX();
        units::radians GetDriverRightY();
        units::radians GetOperatorLeftX();
        units::radians GetOperatorLeftY();
        units::radians GetOperatorRightX();
        units::radians GetOperatorRightY();

        // get button values
        

    private:
        bool ConfigureHardware();
        bool _hardwareConfigured;

        double LEFT_X_ZERO;
        double LEFT_Y_ZERO;
        double RIGHT_X_ZERO;
        double RIGHT_Y_ZERO;

        Feedback _feedback;

        // create the controller objects
        frc::Joystick _driverController;
        frc::Joystick _operatorController;    

        // TODO: make debouncers for indevidual buttons
        // general debouncer for buttons
        frc::Debouncer m_debouncer{50_ms, frc::Debouncer::DebounceType::kBoth};

        frc2::JoystickButton _driverAButton;
        frc2::JoystickButton _driverBButton;
        frc2::JoystickButton _driverXButton;
        frc2::JoystickButton _driverYButton;
};