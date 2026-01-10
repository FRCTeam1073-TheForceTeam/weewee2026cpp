// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/OI.h"
#include <iostream>

OI::OI() :
_driverController(0),
_operatorController(1),
_hardwareConfigured(false)
{
    _hardwareConfigured &= ConfigureHardware();

    if (_hardwareConfigured) {
        std::cerr << "OI hardware configuration error!!" << std::endl;
    }
}


// This method will be called once per scheduler run
void OI::Periodic() {}

double GetDriverLeftX() {
    return _driverController.GetLeftX();
}

double GetDriverLeftY() {
    return _driverController.GetLeftY();
};

double GetDriverRightX() {
    return _driverController.GetRightX();
};

double GetDriverRightY() {
    return _driverController.GetRightY();
};

double GetOperatorLeftX() {
    return _operatorController.GetLeftX();
};

double GetOperatorLeftY() {
    return _operatorController.GetLeftY();
};

double GetOperatorRightX() {
    return _operatorController.GetRightX();
};

double GetOperatorRightY() {
    return _operatorController.GetRightY();
};
