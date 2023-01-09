#pragma once
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/XboxController.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/Joystick.h>
#include "SwerveDrive.hpp"
#include "AHRS.h"
#include <frc/Encoder.h>

#include <math.h>

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include <frc/controller/PIDController.h>

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
    int getLime(); //doesn't have to be void

    
    
    frc::XboxController xboxController{1};
    Joystick joystickController{0};
    Joystick buttonBoard{2};

    double firstDistance = 10;
    double currentDistance;
    double limeOffset;

    double frontRightDriveEncoder = 0;
    double frontLeftDriveEncoder = 0;
    double backRightDriveEncoder = 0;
    double backLeftDriveEncoder = 0;


    double frontRightAngle = 0;
	double frontLeftAngle = 0;
    double backRightAngle = 0;
    double backLeftAngle = 0;

    double tx = 0;
    double targetOffsetAngle_Vertical = 0;

    double encDistance = 52*3.141529/174080; //encoder should be 2048

	WPI_TalonSRX frontLeftDriveMotor = 20;
	WPI_TalonSRX frontLeftAngleMotor = 21;

	WPI_TalonSRX frontRightDriveMotor = 10;
	WPI_TalonSRX frontRightAngleMotor = 11;

	WPI_TalonSRX backLeftDriveMotor = 30;
	WPI_TalonSRX backLeftAngleMotor = 31;

	WPI_TalonSRX backRightDriveMotor = 40;
	WPI_TalonSRX backRightAngleMotor = 41;

    WPI_TalonSRX shooter = 50;
    WPI_TalonSRX feeder = 51;
    WPI_VictorSPX intake = 52;

    
    WPI_VictorSPX leftClimb = 60;
    WPI_VictorSPX rightClimb = 61;
    
    WPI_TalonSRX lift = 5; //change number

private:
    //CONTROLLER VALUES
    double xOffSet;
    bool targetFound = false;
    bool climbSet = false;
    double finalShotVel = 0;
    double finalShotSpeed = 0;
    Timer m_timer;

    frc2::PIDController limelight{0.0275, 0, 0}; //possibly change the pid to 0.025
    frc2::PIDController limelightAuton{0.03, 0, 0}; //possibly change the pid 0.05
    
    double speedMul = 0;
    AHRS *ahrs;
    double nav_yaw = 0;

    int currentState = 0;

    double driveX, driveY, driveZ = 0;
    double maxSpeed = 0;

    SwerveDrive driveTrain;
    I2C arduino {I2C::Port::kOnboard, 0x03};
    
    void updateAllEncoders()
{
    frontRightAngle = frontRightAngleMotor.GetSelectedSensorPosition() * ENCODER_TO_ANGLE;
    frontLeftAngle = frontLeftAngleMotor.GetSelectedSensorPosition() * ENCODER_TO_ANGLE; 
    backRightAngle = backRightAngleMotor.GetSelectedSensorPosition() * ENCODER_TO_ANGLE;
    backLeftAngle = backLeftAngleMotor.GetSelectedSensorPosition() * ENCODER_TO_ANGLE;

    frontRightDriveEncoder = frontRightDriveMotor.GetSelectedSensorPosition();
    frontLeftDriveEncoder = frontLeftDriveMotor.GetSelectedSensorPosition();
    backRightDriveEncoder = backRightDriveMotor.GetSelectedSensorPosition();
    backLeftDriveEncoder = backLeftDriveMotor.GetSelectedSensorPosition();
}

void resetAllEncoders()
{
    frontRightAngleMotor.SetSelectedSensorPosition(0);
    frontLeftAngleMotor.SetSelectedSensorPosition(0);
    backRightAngleMotor.SetSelectedSensorPosition(0);
    backLeftAngleMotor.SetSelectedSensorPosition(0);

    frontRightDriveMotor.SetSelectedSensorPosition(0);
    frontLeftDriveMotor.SetSelectedSensorPosition(0);
    backRightDriveMotor.SetSelectedSensorPosition(0);
    backLeftDriveMotor.SetSelectedSensorPosition(0);
}

};