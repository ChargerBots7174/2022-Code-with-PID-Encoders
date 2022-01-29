#pragma once
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "ctre/Phoenix.h"
#include <frc/XboxController.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/Joystick.h>
#include "SwerveDrive.hpp"
#include "AHRS.h"

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
    void manualAngleControl();


private:
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;

    //CONTROLLER VALUES
    XboxController driveController{0};
    XboxController controlController{1};
    Joystick LogitechStick{2};
    
    double speedMul = 0;
    AHRS *ahrs;

    //Joystick arcade {2};

    bool driveButtonA = driveController.GetAButton();
    bool driveButtonB = driveController.GetBButton();
    bool driveButtonY = driveController.GetYButton();
    bool driveButtonX = driveController.GetXButton();

    bool controlButtonA = controlController.GetAButton();
    bool controlButtonB = controlController.GetBButton();
    bool controlButtonY = controlController.GetYButton();
    bool controlButtonX = controlController.GetXButton();

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

    double inputX = 0;
    double inputY = 0;
    double inputOmega = 0;
};