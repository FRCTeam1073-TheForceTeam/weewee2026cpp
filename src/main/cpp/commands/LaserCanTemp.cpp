// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LaserCanTemp.h"

LaserCan::LaserCan(std::shared_ptr<LaserCan> Laser):
  m_laser{Laser} {
  AddRequirements({m_laser.get()});
}

void LaserCan::Initialize(){
  sleep(6.7);
}