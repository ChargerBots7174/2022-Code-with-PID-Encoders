// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once
// #define PRACTICE
#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "ctre/Phoenix.h"
#include <frc/XboxController.h>
#include <frc/Timer.h>
#include <frc/Servo.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
#include <frc/I2C.h>
//pathweaver
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
//#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include "SwerveDrive.hpp"
#include "AHRS.h"


//std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

//

using namespace frc;

class Robot : public frc::TimedRobot {
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

    //PowerDistributionPanel PDP{0};

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

// 


#ifdef PRACTICE
    
    //practice.
    // WPI_TalonSRX shooterMotor{2}; 
    // WPI_TalonSRX feederMotor {0}; 
    // WPI_TalonSRX climberMotor{0};  
    // WPI_TalonSRX elevatorMotor{0};
    // WPI_TalonSRX intakeMotor{0};
    // WPI_TalonSRX wheelOfFortuneMotor{0};
    // Servo shooterAngleServo {9};


    // WPI_TalonSRX frontLeftMotor{4}; 
    // WPI_TalonSRX backLeftMotor{3};  
    // SpeedControllerGroup leftSideMotors{frontLeftMotor, backLeftMotor};

    // WPI_TalonSRX frontRightMotor{1};  
    // WPI_TalonSRX backRightMotor{2}; 
    // SpeedControllerGroup rightSideMotors{frontRightMotor, backRightMotor};

    // DifferentialDrive drivetrain{leftSideMotors, rightSideMotors};


    
//
#else//main robot
//DRIVETRAIN
        WPI_TalonSRX frontLeftDriveMotor = 20;
		WPI_TalonSRX frontLeftAngleMotor = 21;

		WPI_TalonSRX frontRightDriveMotor = 10;
		WPI_TalonSRX frontRightAngleMotor = 11;

		WPI_TalonSRX backLeftDriveMotor = 30;
		WPI_TalonSRX backLeftAngleMotor = 31;

		WPI_TalonSRX backRightDriveMotor = 40;
		WPI_TalonSRX backRightAngleMotor = 41;

//     WPI_TalonSRX frontLeftMotor{1}; 
//     WPI_TalonSRX backLeftMotor{2};  
//     SpeedControllerGroup leftSideMotors{frontLeftMotor, backLeftMotor};
    
//     WPI_TalonSRX frontRightMotor{3};  
//     WPI_TalonSRX backRightMotor{4}; 
//     SpeedControllerGroup rightSideMotors{frontRightMotor, backRightMotor};

//     DifferentialDrive drivetrain{leftSideMotors, rightSideMotors};
// //
//ACTIONARY MOTORS
    WPI_TalonSRX shooterMotor{15}; 
    WPI_TalonSRX feederMotor{9}; 
    WPI_TalonSRX intakeMotor{8};//9
    WPI_TalonSRX intakeLiftMotor{100};//10
    WPI_TalonSRX wheelOfFortuneMotor {11};
    Servo shooterAngleServoLeft {5};
    Servo shooterAngleServoRight {6};

//
#endif

//DRIVETRAIN STATES
    bool aunkrModeState = false;
    bool rawModeState = false;
    bool climbModeState = false;
    bool ballTrackState = false;
    double driveSpeed = 0;
    double turnSpeed = 0;
//

    Timer time;
    double buttonPressedTime = 0;
    double prevFeedTime = 0.0;
    double feedOffTime = 1.0;
    double feedOnTime = 1.0;
    double feedElpasedTime = 0.0;
    bool feedON = false;
    
    uint8_t buffer = 0;
    I2C arduino {I2C::Port::kOnboard, 0x03};

//Best shooter Values
    double centerPortHeight = 8.225;
    double cameraHeight = 40.0 / 12.0;
    double cameraAngleRAD = 17 * M_PI / 180.0;
    double tYTargetDEG = 0;
    double distanceFromWallFT = 0;
    double cameraToBumper = 15.5 / 12.0;
    
    // std::array<double, 6> targetDistanceFtArray = {0,1,2,3,4,5};
    // std::array<double, 6> shooterAngleArray = {-1,0,0,0,0,1};
    double targetDistanceFtArray[6] = {6,5,4,3,2,1};
    double shooterAngleArray[6] = {-1,0,0,0,0,1};

    double shooterAngle = 0.345000;//manual control of servo//.7 angle servo init line inner port
    double shooterAngleTrench = -0.67;//.68 angle servo trench
//

//Auton Values.
    int autonState = 1;
    double encoderCPR = 2048.0 * 10.71; //one wheel rotation
    double wheelDiameterFt = 0.5; //6 inches
    double rightDistanceTraveledFt = 0.0;
    double leftDistanceTraveledFt = 0.0;
    double goalDistance = 3.0; 
    double leftEncoderCount = 0;
    double rightEncoderCount = 0;
    // double prevLeftEncoderCount = frontLeftMotor.GetSelectedSensorPosition();
    // double prevRightEncoderCount = frontRightMotor.GetSelectedSensorPosition();
    double prevRightX = 0;
    double prevLeftY = 0;

    double goalTargetPosition = 0;
//
    double Tf = 0.30;
    double Ts = 1.0/50.0;
    double a = Ts/(Tf + Ts);

    double driveSpeedd = 0;
    double driveAngle = 0;


    double sensorPos = 0;
   
		PigeonIMU gyro = (0);
        //AnalogGyro a_gyro{1};

    double inputX = 0;
    double inputY = 0;
    double inputOmega = 0;

    double shooterValue = 0;
    

    SwerveDrive driveTrain;

};