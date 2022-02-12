#include "Robot.h"
#include "SwerveDrive.hpp"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#define _USE_MATH_DEFINES

using namespace frc;

void Robot::RobotInit() 
{
    ahrs = new AHRS(SPI::Port::kMXP);
    resetSensors();
}


void Robot::AutonomousInit() 
{
    resetSensors();
}

void Robot::TeleopInit() 
{
    resetSensors();
}

void Robot::TeleopPeriodic() 
{
    nav_yaw = -ahrs->GetYaw();
    //joystick
    driveX = joystickController.GetX();
    driveY = joystickController.GetY();
    driveZ = joystickController.GetTwist();
    maxSpeed = joystickController.GetThrottle();
    //xbox controller
    //driveX = xboxController.GetLeftX();
    //driveY = xboxController.GetLeftY();
    //driveZ = xboxController.GetRightX();
    frc::SmartDashboard::PutNumber("xv", driveX);
    frc::SmartDashboard::PutNumber("yv", driveY);
    frc::SmartDashboard::PutNumber("twist", driveZ);

    driveTrain.drive(driveX, driveY, driveZ, nav_yaw, 1);
}

void Robot::AutonomousPeriodic() 
{
    nav_yaw = -ahrs->GetYaw();
    if(driveTrain.driveDistance(60,0)){ 
        driveTrain.drive(0,0,0,nav_yaw,1);
    }else{
        driveTrain.driveAutonomous(0,-.5,0,nav_yaw,1);
    }
};

void Robot::resetSensors()
{
    ahrs->Reset();
    ahrs->ZeroYaw();
    driveTrain.resetAllEncoders();
}

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