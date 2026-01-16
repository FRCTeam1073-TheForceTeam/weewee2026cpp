# WeeWee2026 C++


# CANBus Configuration

This season we're going to leave lower CANBus IDs for system-wide / special
components and start CANBus IDs for subsystem hardware at ID 5.


| Device Name        | CANBusID | CanBus     |
| ------------------ | -------- | ---------- |
| PDH                |    ?     |  rio       |
| CANdle LED Control |    4     |  rio       |
| Pigeon2 IMU        |    5     |  rio       |
| Swerve 0: Encoder  |    6     |  rio       |
| Swerve 0: Drive    |    7     |  rio       |
| Swerve 0: Steer    |    8     |  rio       |
| Swerve 1: Encoder  |    9     |  rio       |
| Swerve 1: Drive    |    10    |  rio       |
| Swerve 1: Steer    |    11    |  rio       |
| Swerve 2: Encoder  |    12    |  rio       |
| Swerve 2: Drive    |    13    |  rio       |
| Swerve 2: Steer    |    14    |  rio       |
| Swerve 3: Encoder  |    15    |  rio       |
| Swerve 3: Drive    |    16    |  rio       |
| Swerve 3: Steer    |    17    |  rio       |


# OI (Driver Controller)

| Button/Joystick  | Command Name          |
| ---------------- | --------------------- |
| Left Joystick X  | GetDriverLeftX        |
| Left Joystick Y  | GetDriverLeftY        |
| Right Joystick X | GetDriverRightX       |
| Right Joystick Y | GetDriverRightY       |
| Left Trigger     | GetDriverLeftTrigger  |
| Right Trigger    | GetDriverRightTrigger |
| A Button         | GetDriverAButton      |
| B Button         | GetDriverBButton      |
| X Button         | GetDriverXButton      |
| Y Button         | GetDriverYButton      |
| Menu Button      | GetDriverMenuButton   |
| View Button      | GetDriverViewButton   |
| Left Bumper      | GetDriverLeftBumper   |
| Right Bumper     | GetDriverRightBumper  |
| D-Pad Up         | GetDriverDPadUp       |
| D-Pad Right      | GetDriverDPadRight    |
| D-Pad Left       | GetDriverDPadLeft     |
| D-Pad Down       | GetDriverDPadDown     |
| Zero Controller  | ZeroDriverController  |





# OI (Operator Controller)

| Button/Joystick  | Command Name            |
| ---------------- | ----------------------- |
| Left Joystick X  | GetOperatorLeftX        |
| Left Joystick Y  | GetOperatorLeftY        |
| Right Joystick X | GetOperatorRightX       |
| Right Joystick Y | GetOperatorRightY       |
| Left Trigger     | GetOperatorLeftTrigger  |
| Right Trigger    | GetOperatorRightTrigger |
| A Button         | GetOperatorAButton      |
| B Button         | GetOperatorBButton      |
| X Button         | GetOperatorXButton      |
| Y Button         | GetOperatorYButton      |
| Menu Button      | GetOperatorMenuButton   |
| View Button      | GetOperatorViewButton   |
| Left Bumper      | GetOperatorLeftBumper   |
| Right Bumper     | GetOperatorRightBumper  |
| D-Pad Up         | GetOperatorDPadUp       |
| D-Pad Right      | GetOperatorDPadRight    |
| D-Pad Left       | GetOperatorDPadLeft     |
| D-Pad Down       | GetOperatorDPadDown     |
| Zero Controller  | ZeroOperatorController  |





Add mechanisms after this...
