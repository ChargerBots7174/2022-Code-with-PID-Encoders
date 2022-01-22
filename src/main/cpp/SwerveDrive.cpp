#include "SwerveDrive.hpp"  


SwerveDrive::SwerveDrive(){
        
}
	
SwerveDrive::~SwerveDrive()
{
	
}

void SwerveDrive::drive(double xv,double yv,double omega,double speedMul){
    // ChassisSpeeds speeds;
  frontRightAngle = frontRightAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
  frontLeftAngle = frontLeftAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
  backRightAngle = backRightAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
  backLeftAngle = backLeftAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
    double r = sqrt ((robotLength * robotLength) + (robotWidth * robotWidth));
    yv *= -1;

if (yv < 0.075 && yv > -0.075){
yv = 0;
}
if (xv < 0.075 && xv > -0.075){
xv = 0;
}
if (omega < 0.075 && omega > -0.075){
omega = 0;

        gyroPID.EnableContinuousInput(-180, 180);
        gyroMul = gyroPID.Calculate(speedMul, 0);
}
        //gyro.GetYawPitchRoll(ypr);
        z = speedMul;//ypr[0];
        robotHeading = (-speedMul * 2*acos(0)) / 180;

    // xv = (xv)+0.03;//.00975
    // yv = yv;

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

     
// if(yv >= 0){

//            backRightSpeed = backRightSpeed;
//      backLeftSpeed = backLeftSpeed;
//      frontRightSpeed = frontRightSpeed;
//      frontLeftSpeed = frontLeftSpeed;

//     //MOTOR ANGLE CALCULATIONS
//      backLeftAngleCalc = 180 * atan2 (a, d) / (2*acos(0));
//      backRightAngleCalc = 180 * atan2 (a, c) / (2*acos(0));
//      frontLeftAngleCalc = 180 * atan2 (b, d) / (2*acos(0));
//      frontRightAngleCalc = 180 * atan2 (b, c) / (2*acos(0));
// }else{
//      backRightSpeed = -backRightSpeed;
//      backLeftSpeed = -backLeftSpeed;
//      frontRightSpeed = -frontRightSpeed;
//      frontLeftSpeed = -frontLeftSpeed;
// }

//     //MOTOR ANGLE CALCULATIONS
//     if (xv > 0.075 && yv < -0.075){
//      backLeftAngleCalc = (180 * atan2 (a, d) / (2*acos(0))) - 180;
//      backRightAngleCalc = (180 * atan2 (a, c) / (2*acos(0))) - 180;
//      frontLeftAngleCalc = (180 * atan2 (b, d) / (2*acos(0))) - 180;
//      frontRightAngleCalc = (180 * atan2 (b, c) / (2*acos(0))) - 180;    

//      }else if (xv < 0.075 && yv < -0.075){
//      backLeftAngleCalc = (180 * atan2 (a, d) / (2*acos(0))) + 180;
//      backRightAngleCalc = (180 * atan2 (a, c) / (2*acos(0))) + 180;
//      frontLeftAngleCalc = (180 * atan2 (b, d) / (2*acos(0))) + 180;
//      frontRightAngleCalc = (180 * atan2 (b, c) / (2*acos(0))) + 180;           
// }
     



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

    frontLeftDriveMotor.Set(ControlMode::PercentOutput, (frontLeftSpeed * 0.6));//////////////////////////////////////////////////////////speed * 0.6 multi
    frontRightDriveMotor.Set(ControlMode::PercentOutput, (frontRightSpeed * 0.6));
    backRightDriveMotor.Set(ControlMode::PercentOutput, (backRightSpeed * 0.6));
    backLeftDriveMotor.Set(ControlMode::PercentOutput, (backLeftSpeed * 0.6));

        steerPID.EnableContinuousInput(-180, 180);
        drivePID.EnableContinuousInput(-1, 1);

    frontLeftAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(frontLeftAngle, frontLeftAngleCalc)));
    frontRightAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(frontRightAngle,frontRightAngleCalc)));
    backRightAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(backRightAngle, backRightAngleCalc)));
    backLeftAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(backLeftAngle, backLeftAngleCalc)));
}