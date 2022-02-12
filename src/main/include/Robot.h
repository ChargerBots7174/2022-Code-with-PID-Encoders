#pragma once
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/XboxController.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/Joystick.h>
#include "SwerveDrive.hpp"
#include "AHRS.h"
#include <frc/Encoder.h>

using namespace frc;

class Robot : public frc::TimedRobot 
{
 public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void resetSensors();


private:
    //CONTROLLER VALUES
    XboxController xboxController{0};
    Joystick joystickController{0};
    
    double speedMul = 0;
    AHRS *ahrs;
    double nav_yaw = 0;

    int currentState = 0;

    double driveX, driveY, driveZ = 0;
    double maxSpeed = 0;

    WPI_TalonSRX frontLeftDriveMotor = 20;
    WPI_TalonSRX frontLeftAngleMotor = 21;

    WPI_TalonSRX frontRightDriveMotor = 10;
    WPI_TalonSRX frontRightAngleMotor = 11;

    WPI_TalonSRX backLeftDriveMotor = 30;
    WPI_TalonSRX backLeftAngleMotor = 31;

    WPI_TalonSRX backRightDriveMotor = 40;
    WPI_TalonSRX backRightAngleMotor = 41;

    SwerveDrive driveTrain;
    I2C arduino {I2C::Port::kOnboard, 0x03};
};