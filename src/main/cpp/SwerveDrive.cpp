#include "SwerveDrive.hpp"  


SwerveDrive::SwerveDrive() {}
SwerveDrive::~SwerveDrive() {}

void SwerveDrive::drive(double xv,double yv,double omega,double speedMul)
{
    updateAllEncoders();
    double r = sqrt ((robotLength * robotLength) + (robotWidth * robotWidth));
    yv *= -1;

    if (yv < 0.075 && yv > -0.075)
    {
        yv = 0;
    }
    if (xv < 0.075 && xv > -0.075)
    {
        xv = 0;
    }
    if (omega < 0.075 && omega > -0.075)
    {
        omega = 0;
        gyroPID.EnableContinuousInput(-180, 180);
        gyroMul = gyroPID.Calculate(speedMul, 0);
    }
    z = speedMul;
    robotHeading = (-speedMul * 2*acos(0)) / 180;

    double temp = yv * cos(robotHeading) + xv * sin(robotHeading); 
    xv = -yv * sin(robotHeading) + xv * cos(robotHeading); 
    yv = temp;

    double a = xv - omega * (robotLength / r);
    double b = xv + omega * (robotLength / r);
    double c = yv - omega * (robotWidth / r);
    double d = yv + omega * (robotWidth / r);
    
    //MOTOR SPEED CALCULATIONS
    backRightSpeed = sqrt ((a * a) + (d * d));
    backLeftSpeed = sqrt ((a * a)) + ((c * c));
    frontRightSpeed = sqrt ((b * b)) + ((d * d));
    frontLeftSpeed = sqrt ((b * b)) + ((c * c));

    //MOTOR ANGLE CALCULATIONS
    backLeftAngleCalc = 180 * atan2 (a, d) / (2*acos(0));
    backRightAngleCalc = 180 * atan2 (a, c) / (2*acos(0));
    frontLeftAngleCalc = 180 * atan2 (b, d) / (2*acos(0));
    frontRightAngleCalc = 180 * atan2 (b, c) / (2*acos(0));

    //SET DRIVE MOTORS
    frontLeftDriveMotor.Set(ControlMode::PercentOutput, (frontLeftSpeed * 0.6));
    frontRightDriveMotor.Set(ControlMode::PercentOutput, (frontRightSpeed * 0.6));
    backRightDriveMotor.Set(ControlMode::PercentOutput, (backRightSpeed * 0.6));
    backLeftDriveMotor.Set(ControlMode::PercentOutput, (backLeftSpeed * 0.6));

    steerPID.EnableContinuousInput(-180, 180);
    drivePID.EnableContinuousInput(-1, 1);

    //SET ANGLE MOTORS
    frontLeftAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(frontLeftAngle, frontLeftAngleCalc)));
    frontRightAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(frontRightAngle,frontRightAngleCalc)));
    backRightAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(backRightAngle, backRightAngleCalc)));
    backLeftAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(backLeftAngle, backLeftAngleCalc)));

    /* SMART DASHBOARD */
    // frc::SmartDashboard::PutNumber("xv", xv);
    // frc::SmartDashboard::PutNumber("yv", yv);
    // frc::SmartDashboard::PutNumber("backRightSpeed", backRightSpeed);
    // frc::SmartDashboard::PutNumber("backLeftspeed", backLeftSpeed);
    // frc::SmartDashboard::PutNumber("front right speed", frontRightSpeed);
    // frc::SmartDashboard::PutNumber("front left speed", frontLeftSpeed);
    frc::SmartDashboard::PutNumber("gyro", z);
    frc::SmartDashboard::PutNumber("head", robotHeading);
    frc::SmartDashboard::PutNumber("speedMul", speedMul);
    frc::SmartDashboard::PutNumber("gyroMul", gyroMul);

    frc::SmartDashboard::PutNumber("backRightAngle", backRightAngleCalc);
    frc::SmartDashboard::PutNumber("backLeftAngleCalc", backLeftAngleCalc);
    frc::SmartDashboard::PutNumber("front right AngleCalc", frontRightAngleCalc);
    frc::SmartDashboard::PutNumber("front left AngleCalc", frontLeftAngleCalc);

    frc::SmartDashboard::PutNumber("backRightAngle", backRightAngle);
    frc::SmartDashboard::PutNumber("backLeftAngle", backLeftAngle);
    frc::SmartDashboard::PutNumber("front right Angle", frontRightAngle);
    frc::SmartDashboard::PutNumber("front left Angle", frontLeftAngle);

}


void SwerveDrive::updateAllEncoders(){
    frontRightAngle = frontRightAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
    frontLeftAngle = frontLeftAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
    backRightAngle = backRightAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
    backLeftAngle = backLeftAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
}

void SwerveDrive::resetAllEncoders(){
    frontRightAngleMotor.SetSelectedSensorPosition(0);
    frontLeftAngleMotor.SetSelectedSensorPosition(0);
    backRightAngleMotor.SetSelectedSensorPosition(0);
    backLeftAngleMotor.SetSelectedSensorPosition(0);

    frontRightDriveMotor.SetSelectedSensorPosition(0);
    frontLeftDriveMotor.SetSelectedSensorPosition(0);
    backRightDriveMotor.SetSelectedSensorPosition(0);
    backLeftDriveMotor.SetSelectedSensorPosition(0);
}