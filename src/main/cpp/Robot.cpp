#include "Robot.h"
#include "SwerveDrive.hpp"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#define _USE_MATH_DEFINES

using namespace frc;

void Robot::RobotInit() 
{
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    ahrs = new AHRS(SPI::Port::kMXP);
    ahrs->Reset();
}


void Robot::AutonomousInit() 
{
}

void Robot::TeleopInit() 
{
    ahrs->Reset();
    ahrs->ZeroYaw();
    driveTrain.resetAllEncoders();
}

void Robot::TeleopPeriodic() 
{
    nav_yaw = -ahrs->GetYaw();
    //joystick
    driveX = joystickController.GetX();
    driveY = joystickController.GetY();
    driveZ = joystickController.GetTwist();
    //xbox controller
    // driveX = driveController.GetLeftX();
    // driveY = driveController.GetLeftY();
    // driveZ = driveController.GetRightX();

    driveTrain.drive(driveX, driveY, driveZ, nav_yaw);
}

void Robot::AutonomousPeriodic() {}
void Robot::RobotPeriodic() {}
void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif