#ifndef SWERVEDRIVE_H
#define SWERVEDRIVE_H
#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
//#include <frc/SpeedControllerGroup.h>
#include <hal/SimDevice.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include "cmath"
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

using namespace frc;


class SwerveDrive  
{
	private:
		// Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
		// Translation2d m_frontRightLocation{0.381_m, -0.381_m};
		// Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
		// Translation2d m_backRightLocation{-0.381_m, -0.381_m};

		// SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

		// ChassisSpeeds speeds{1_mps, 3_mps, 1.5_rad_per_s};

	
	public:
		WPI_TalonSRX frontLeftDriveMotor = 20;
		WPI_TalonSRX frontLeftAngleMotor = 21;

		WPI_TalonSRX frontRightDriveMotor = 10;
		WPI_TalonSRX frontRightAngleMotor = 11;

		WPI_TalonSRX backLeftDriveMotor = 30;
		WPI_TalonSRX backLeftAngleMotor = 31;

		WPI_TalonSRX backRightDriveMotor = 40;
		WPI_TalonSRX backRightAngleMotor = 41;

		//PigeonIMU gyro = (0);


		frc2::PIDController steerPID{0.00675, 0, 0};
		frc2::PIDController drivePID{1, 0, 0};
		frc2::PIDController gyroPID{0.045, 0, 0};


		double robotWidth = 30;
		double robotLength = 30;
		double z = 0;
		double ypr[3];

		double gyroMul = 0;

		void drive(double xv, double yv, double omega, double speedMul);
		SwerveDrive();
		~SwerveDrive();

		double frontRightAngle = 0;
		double frontLeftAngle = 0;
    	double backRightAngle = 0;
    	double backLeftAngle = 0;

		  double robotHeading = 0;

		  double backRightSpeed = 0;
    double backLeftSpeed = 0;
    double frontRightSpeed = 0;
    double frontLeftSpeed = 0;

	double backRightAngleCalc = 0;
    double backLeftAngleCalc = 0;
    double frontRightAngleCalc = 0;
    double frontLeftAngleCalc = 0;

	 double OldbackRightSpeed =0;
     double OldbackLeftSpeed = 0;
    double OldfrontRightSpeed = 0;
    double OldfrontLeftSpeed = 0;

	XboxController controller3{3};
	double setOmega = 0;
};
#endif