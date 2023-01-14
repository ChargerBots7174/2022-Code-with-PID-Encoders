#include "Robot.h"
#include "SwerveDrive.hpp"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include "frc/DigitalInput.h"

#include <frc/TimedRobot.h>
#include <frc/Timer.h>

#include <cameraserver/CameraServer.h>

#define _USE_MATH_DEFINES

// using namespace frc;

void Robot::RobotInit()
{
    ahrs = new AHRS(SPI::Port::kMXP);
    resetSensors();
    frc::CameraServer::StartAutomaticCapture();
}

void Robot::AutonomousInit()
{
    lift.SetSelectedSensorPosition(0);
    resetSensors();
    resetAllEncoders();
    m_timer.Reset();
    m_timer.Start();
    targetFound = false;
}

void Robot::TeleopInit()
{

    resetSensors();
    resetAllEncoders();
    lift.SetSelectedSensorPosition(0);
    climbSet = false;
}


void Robot::TeleopPeriodic()
{
    frc::SmartDashboard::PutNumber("back left vel", backLeftDriveMotor.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("front left vel", frontLeftDriveMotor.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("back right vel", backRightDriveMotor.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("front right vel", frontRightDriveMotor.GetSelectedSensorVelocity());
    // frc::SmartDashboard::PutNumber("intake en", lift.GetSelectedSensorPosition());
    // //INTAKE UP AND DOWN
    // frc::DigitalInput IntakeSwitch{0};
    // bool moveIntake = IntakeSwitch.Get();
    // frc::SmartDashboard::PutNumber("Switch", IntakeSwitch.Get());

    frc::SmartDashboard::PutNumber("Lift Enc", lift.GetSelectedSensorPosition());
    if ((joystickController.GetRawButton(10) > 0.5 || buttonBoard.GetRawButton(1) > 0.5) && lift.GetSelectedSensorPosition() < 200000)
    { // false to true
        lift.Set(ControlMode::PercentOutput, 0.3);
    }
    else if ((joystickController.GetRawButton(9) > 0.5 || buttonBoard.GetRawButton(2) > 0.5) && lift.GetSelectedSensorPosition() > 0)
    {
        lift.Set(ControlMode::PercentOutput, -0.3);
    }
    else
    {
        lift.Set(ControlMode::PercentOutput, 0.0);
    }
    // END

    // LIMELIGHT MATH
    //double targetOffsetAngle_Vertical = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0) - 2;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 26.2; // 24.65 //26.75

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 18.0;

    // distance from the target to the floor
    double goalHeightInches = 104.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians);
    // frc::SmartDashboard::PutNumber("Goal Distance", distanceFromLimelightToGoalInches);
    // END

    // DRIVE
    nav_yaw = -ahrs->GetYaw();
    // double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0) - 2;
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 1);
    // frc::SmartDashboard::PutNumber("CitrusLumen", tx);

    if (joystickController.GetRawButton(3) > 0.5 || buttonBoard.GetRawButton(9) > 0.5 || buttonBoard.GetRawButton(10) > 0.5)
    { // changed from 4 to 3
        driveX = 0;
        driveY = 0;
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);
        tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0) - 2;
        targetOffsetAngle_Vertical = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0) - 2;
        limelight.EnableContinuousInput(-300, 300);
        driveZ = -limelight.Calculate(tx, 0);
    }
    else if (joystickController.GetRawButton(2) == 0)
    {
        driveX = joystickController.GetX() * 0.25;
        driveY = joystickController.GetY() * 0.25;
        if (joystickController.GetRawButton(7) > 0.5)
        { // changed 8 to 7
            driveZ = 0;
        }
        else
        {
            driveZ = joystickController.GetTwist() * 0.25;
        }
        maxSpeed = 1;
    }
    else if (joystickController.GetTrigger() > 0.5)
    {
        driveX = joystickController.GetX() * 0.35;
        driveY = joystickController.GetY() * 0.35;
        if (joystickController.GetRawButton(7) > 0.5)
        { // changed 8 to 7
            driveZ = 0;
        }
        else
        {
            driveZ = joystickController.GetTwist() * 0.35;
        }
        maxSpeed = 1;
    }
    else
    {
        driveX = joystickController.GetX();
        driveY = joystickController.GetY();
        if (joystickController.GetRawButton(7) > 0.5)
        { // changed 8 to 7
            driveZ = 0;
        }
        else
        {
            driveZ = joystickController.GetTwist();
        }
        maxSpeed = 1;
    }
    // END

    // FEEDER
    if (xboxController.GetLeftTriggerAxis() > 0.01 || buttonBoard.GetRawButton(3) > 0.5)
    { // watch out
        feeder.Set(ControlMode::PercentOutput, 0.5);
    }
    else if (xboxController.GetLeftBumper() > 0.5 || buttonBoard.GetRawButton(4) > 0.5)
    {
        feeder.Set(ControlMode::PercentOutput, -0.50);
    }
    else
    {
        feeder.Set(ControlMode::PercentOutput, 0.0);
    }
    // END

    // INTAKE
    if (xboxController.GetAButton() > 0.5 || buttonBoard.GetRawButton(5) > 0.5)
    {
        intake.Set(ControlMode::PercentOutput, 0.6);
    }
    else if (xboxController.GetYButton() > 0.5 || buttonBoard.GetRawButton(6) > 0.5)
    {
        intake.Set(ControlMode::PercentOutput, -0.6);
    }
    else
    {
        intake.Set(ControlMode::PercentOutput, 0.0);
    }
    // END

    // SHOOTER
    if ((xboxController.GetRightBumper() > 0.5 || buttonBoard.GetRawButton(7) > 0.5) && lift.GetSelectedSensorPosition() < 5000)
    { // LIMELIGHT SHOOTER
        // shooter.Set(ControlMode::PercentOutput, 0.265); //close shot
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);
        targetOffsetAngle_Vertical = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
        finalShotSpeed = 0.0014 * (distanceFromLimelightToGoalInches) + 0.3139;
        finalShotVel = 55.556 * (distanceFromLimelightToGoalInches) + 2555.6;
        shooter.Set(ControlMode::PercentOutput, finalShotSpeed + 0.05);
        if (shooter.GetSelectedSensorVelocity() > finalShotVel - 1150)
        {
            feeder.Set(ControlMode::PercentOutput, -1);
        }
    }
    else if (xboxController.GetRightTriggerAxis() > 0.5)
    { // LONG DISTANCE
        shooter.Set(ControlMode::PercentOutput, 0.69);
        if (shooter.GetSelectedSensorVelocity() > 13800)
        {
            feeder.Set(ControlMode::PercentOutput, -1);
        }
    }
    else if (xboxController.GetXButton() > 0.5)
    { // SHORT DISTANCE
        shooter.Set(ControlMode::PercentOutput, 0.555);
        if (shooter.GetSelectedSensorVelocity() > 11200)
        {
            feeder.Set(ControlMode::PercentOutput, -1);
        }
    }
    else if ((xboxController.GetBButton() > 0.5 || buttonBoard.GetRawButton(8) > 0.5) && lift.GetSelectedSensorPosition() < 5000)
    { // SHOOTER NO LIMELIGHT VERY SHORT
        shooter.Set(ControlMode::PercentOutput, 0.265);
        if (shooter.GetSelectedSensorVelocity() > 4800)
        {
            feeder.Set(ControlMode::PercentOutput, -1);
        }
    }
    else
    {
        shooter.Set(ControlMode::PercentOutput, 0.0);
    }
    // frc::SmartDashboard::PutNumber("finalShootVel", finalShotVel);
    // frc::SmartDashboard::PutNumber("finalShootSpeed", finalShotSpeed);

    // CLIMBERS
    if (joystickController.GetRawButton(4) > 0.5 || buttonBoard.GetY() < -0.5)
    { // DOWN CLIMBER MANUAL   //changed to 3 to 4
        leftClimb.Set(ControlMode::PercentOutput, -0.75);
        rightClimb.Set(ControlMode::PercentOutput, -0.75);
        climbSet = true;
    }
    else if (joystickController.GetRawButton(5) > 0.5 || buttonBoard.GetY() > 0.5)
    { // UP CLIMBER MANUAL
        leftClimb.Set(ControlMode::PercentOutput, 1.00);
        rightClimb.Set(ControlMode::PercentOutput, 1.0);
        climbSet = false;
    }
    else if (climbSet == true)
    { // DOWN CLIMBER AUTOMATIC
        leftClimb.Set(ControlMode::PercentOutput, -0.1);
        rightClimb.Set(ControlMode::PercentOutput, -0.1);
    }
    else
    {
        leftClimb.Set(ControlMode::PercentOutput, -0.0);
        rightClimb.Set(ControlMode::PercentOutput, -0.0);
    }
    // END

    driveTrain.drive(driveX, driveY, driveZ, nav_yaw, 1);

    // frc::SmartDashboard::PutNumber("xv", driveX);
    // frc::SmartDashboard::PutNumber("yv", driveY);
    // frc::SmartDashboard::PutNumber("twist", driveZ);
    // frc::SmartDashboard::PutNumber("gyro", nav_yaw);
}

void Robot::AutonomousPeriodic()
{

    nav_yaw = -ahrs->GetYaw();

    double targetOffsetAngle_Vertical = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0) - 2;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 26.2; // 24.65

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 18.0;

    // distance from the target to the floor
    double goalHeightInches = 104.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians);
    double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0) - 2;
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);
    frc::SmartDashboard::PutNumber("CitrusLumen", tx);

    driveTrain.drive(driveX, driveY, driveZ, nav_yaw, 1);
    /*
        if(lift.GetSelectedSensorPosition() < 198000) {
            lift.Set(ControlMode::PercentOutput, 0.3);
        }
        else{
            lift.Set(ControlMode::PercentOutput, 0);
        }

    */
    /*START OF OG CODE*/
    if (double(m_timer.Get()) < 0.5)
    {
        driveX = 0;
        driveY = 0.081;
        driveZ = 0;
    }
    else if (double(m_timer.Get()) < 2.5)
    {
        driveX = 0;
        driveY = 0.5;
        driveZ = 0;
        shooter.Set(ControlMode::PercentOutput, 0);
        feeder.Set(ControlMode::PercentOutput, 0);
    }
    else if (double(m_timer.Get()) < 3)
    {
        driveX = 0;
        driveY = 0;
        driveZ = 0;
    }
    else
    {
        driveX = 0;
        driveY = 0;
        limelight.EnableContinuousInput(-300, 300);
        driveZ = -limelightAuton.Calculate(tx, 0);
        if (double(m_timer.Get()) > 5)
        {
            finalShotSpeed = 0.0014 * (distanceFromLimelightToGoalInches) + 0.3139;
            finalShotVel = 55.556 * (distanceFromLimelightToGoalInches) + 2555.6;
            shooter.Set(ControlMode::PercentOutput, finalShotSpeed + 0.05);
            if (shooter.GetSelectedSensorVelocity() > finalShotVel - 1300)
            {
                feeder.Set(ControlMode::PercentOutput, -1);
            }
        }
    }
    // END OF OG CODE */

    /*START OF WEDNESDAY CODE
    if((frontRightDriveMotor.GetSelectedSensorPosition()*52*3.141529/174080) < 40) {
        driveX = 0;
        driveY = -0.4;
        driveZ = 0;
        shooter.Set(ControlMode::PercentOutput, 0);
        feeder.Set(ControlMode::PercentOutput, 0);
        intake.Set(ControlMode::PercentOutput, 0.5);
    }
    else if(nav_yaw > - 140 && targetFound == false){ //double(m_timer.Get()) < 1000
        driveX = 0;
        driveY = 0;
        driveZ = 0.3;
        shooter.Set(ControlMode::PercentOutput, 0);
        feeder.Set(ControlMode::PercentOutput, 0);
        intake.Set(ControlMode::PercentOutput, 0.5);
    }
    else{
        targetFound == true;
        driveX = 0;
        driveY = 0;
        intake.Set(ControlMode::PercentOutput, 0.5);
        limelight.EnableContinuousInput(-300, 300);
        driveZ = -limelightAuton.Calculate(tx, 0);
        if(double(m_timer.Get()) > 8){
            finalShotSpeed = 0.0014*(distanceFromLimelightToGoalInches) + 0.3139;
            finalShotVel = 55.556*(distanceFromLimelightToGoalInches) + 2555.6;
            shooter.Set(ControlMode::PercentOutput, finalShotSpeed+0.05);
            if(shooter.GetSelectedSensorVelocity() > finalShotVel-1500){
                feeder.Set(ControlMode::PercentOutput, -1);
            }
        }
    }

// END OF WEDNESDAY'S CODE */

    /* START OF THURSDAY'S CODE
    if((frontRightDriveMotor.GetSelectedSensorPosition()*52*3.141529/174080) < 40){ //double(m_timer.Get()) < 1000
        driveX = 0;
        driveY = -0.4;
        driveZ = 0;
        shooter.Set(ControlMode::PercentOutput, 0);
        feeder.Set(ControlMode::PercentOutput, 0);
        intake.Set(ControlMode::PercentOutput, 0.5);
    }
    // else if(nav_yaw > - 140 && targetFound == false){ //(frontRightDriveMotor.GetSelectedSensorPosition()*52*3.141529/174080) < 25 //can you change 140?
    //     driveX = 0;
    //     driveY = 0;
    //     driveZ = 0.3;
    //     shooter.Set(ControlMode::PercentOutput, 0);
    //     feeder.Set(ControlMode::PercentOutput, 0);
    //     intake.Set(ControlMode::PercentOutput, 0.5);
    else if((frontRightDriveMotor.GetSelectedSensorPosition()*52*3.141529/174080) > 40 and (frontRightDriveMotor.GetSelectedSensorPosition()*52*3.141529/174080) < 50){ //double(m_timer.Get()) < 1000
        driveX = 0;
        driveY = 0;
        driveZ = 0.3;
        shooter.Set(ControlMode::PercentOutput, 0);
        feeder.Set(ControlMode::PercentOutput, 0);
        intake.Set(ControlMode::PercentOutput, 0.5);
    }
    else{
        driveX = 0;
        driveY = 0;
        intake.Set(ControlMode::PercentOutput, 0.5);
        limelight.EnableContinuousInput(-300, 300);
        driveZ = -limelightAuton.Calculate(tx, 0);
        if(double(m_timer.Get()) > 8){
            finalShotSpeed = 0.0014*(distanceFromLimelightToGoalInches) + 0.3139;
            finalShotVel = 55.556*(distanceFromLimelightToGoalInches) + 2555.6;
            shooter.Set(ControlMode::PercentOutput, finalShotSpeed+0.05);
            if(shooter.GetSelectedSensorVelocity() > finalShotVel-1500){
                feeder.Set(ControlMode::PercentOutput, -1);
            }
        }
    }
    END OF THURSDAY'S CODE*/

    // //START OF THE 5 BALL AUTON
    // //network table for red and blue ball
    // double tx = nt::NetworkTableInstance::GetDefault().GetTable("limeballr")->GetNumber("tx",0.0)-2;
    // nt::NetworkTableInstance::GetDefault().GetTable("limeballr")->PutNumber("ledMode",3);
    // nt::NetworkTableInstance::GetDefault().GetTable("limeballr")->PutNumber("camMode",0);

    // double tx = nt::NetworkTableInstance::GetDefault().GetTable("limeballb")->GetNumber("tx",0.0)-2;
    // nt::NetworkTableInstance::GetDefault().GetTable("limeballb")->PutNumber("ledMode",3);
    // nt::NetworkTableInstance::GetDefault().GetTable("limeballb")->PutNumber("camMode",0);
    // //if the ball isn't in view, turn 90 degrees

    // if(double(m_timer.Get()) < 2500){
    //     feeder.Set(ControlMode::PercentOutput, 0.5);
    //     finalShotSpeed = 0.0014*(distanceFromLimelightToGoalInches) + 0.3139;
    //     finalShotVel = 55.556*(distanceFromLimelightToGoalInches) + 2555.6;
    //     shooter.Set(ControlMode::PercentOutput, finalShotSpeed+0.05);
    //     // feeder.Set(ControlMode::PercentOutput, 0.5);
    //     // shooter.Set(ControlMode::PercentOutput, 0.5);
    // }
    // else if(double(m_timer.Get()) < 5500){ //find them time for the robot to turn 90 degrees
    //     // driveX = 0;
    //     // driveY = 0.5;
    //     // driveZ = 0;
    //     frontRightDriveEncoder = frontRightDriveMotor.GetSelectedSensorPosition();
    //     frontLeftDriveEncoder = frontLeftDriveMotor.GetSelectedSensorPosition();
    //     backRightDriveEncoder = backRightDriveMotor.GetSelectedSensorPosition();
    //     backLeftDriveEncoder = backLeftDriveMotor.GetSelectedSensorPosition();
    //     //go back straight using encoders ?

    //     /*frontLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //     frontRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //     backLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //     backRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);*/
    //     //go back straight
    // }
    // else if(double(m_timer.Get()) < 6500){
    //     frontLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //     frontRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //     backLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //     backRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //     //start 90 degrees turn (change the values)
    //     if (distanceFromLimelightToGoalInches >= 140 /*random distance value*/) {
    //         intake.Set(ControlMode::PercentOutput, 0.5);
    //         frontRightDriveEncoder = frontRightDriveMotor.GetSelectedSensorPosition();
    //         frontLeftDriveEncoder = frontLeftDriveMotor.GetSelectedSensorPosition();
    //         backRightDriveEncoder = backRightDriveMotor.GetSelectedSensorPosition();
    //         backLeftDriveEncoder = backLeftDriveMotor.GetSelectedSensorPosition();
    //     //go forward using encoders
    //         /*frontLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         frontRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         backLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         backRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);*/
    //         //go foward
    //     }
    // }
    // else if((double(m_timer.Get()) < 7500) {
    //     if (distanceFromLimelightToGoalInches >= 140 /*random number need number*/) {
    //         frontLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         frontRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         backLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         backRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         //start 90 degrees turn (change the values)

    //         if((double(m_timer.Get()) < 9500) {
    //         intake.Set(ControlMode::PercentOutput, 0.5);
    //         feeder.Set(ControlMode::PercentOutput, 0.5);
    //         finalShotSpeed = 0.0014*(distanceFromLimelightToGoalInches) + 0.3139;
    //         finalShotVel = 55.556*(distanceFromLimelightToGoalInches) + 2555.6;
    //         shooter.Set(ControlMode::PercentOutput, finalShotSpeed+0.05);
    //         }
    //     }
    // }
    // else if((double(m_timer.Get()) < 10500) {
    //     if (distanceFromLimelightToGoalInches() >= 140 /*random number need number*/) {
    //         frontLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         frontRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         backLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         backRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //         //start 90 degrees turn
    //             else() {
    //                 intake.Set(ControlMode::PercentOutput, 0.5);
    //                 //human player NEEDS to role the ball in front of the robot
    //                 frontLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //                 frontRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //                 backLeftDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //                 backRightDriveMotor.Set(ControlMode::PercentOutput, 0.5);
    //                 //drive straight
    //                 feeder.Set(ControlMode::PercentOutput, 0.5);
    //                 finalShotSpeed = 0.0014*(distanceFromLimelightToGoalInches) + 0.3139;
    //                 finalShotVel = 55.556*(distanceFromLimelightToGoalInches) + 2555.6;
    //                 shooter.Set(ControlMode::PercentOutput, finalShotSpeed+0.05);
    //             }
    //         }
    //     }
    // //END OF THE 5 BALL AUTON

    // double DistanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/tan(angleToGoalRadians);

    // if (distanceFromLimelightToGoalInches > 182) //tests the shooting speed for the shooter to do shooter value during this period

    // if((frontRightDriveMotor.GetSelectedSensorPosition()*52*3.141529/174080) > 50){
    //     driveX = 0;
    //     driveY = 0.5;
    //     driveZ = 0;
    // }
    // else{
    //         shooter.Set(ControlMode::PercentOutput, 0.5);
    //         feeder.Set(ControlMode::PercentOutput, -0.5);
    //         driveX = 0;
    //         driveY = 0;
    //         driveZ = 0;
    // }

    // code for the shooting, do when testing
    // go so many inches->pick up ball->turn around->limelight adjustment->shoot ball
}

// if(driveTrain.driveDistance(60,0)){
//     driveTrain.drive(0,0,0,nav_yaw,1);
// }else{
//     driveTrain.driveAutonomous(0,-.5,0,nav_yaw,1);
// }

void Robot::resetSensors()
{
    ahrs->Reset();
    ahrs->ZeroYaw();
    driveTrain.resetAllEncoders();

    frontRightAngleMotor.SetSelectedSensorPosition(0);
    frontLeftAngleMotor.SetSelectedSensorPosition(0);
    backRightAngleMotor.SetSelectedSensorPosition(0);
    backLeftAngleMotor.SetSelectedSensorPosition(0);

    frontRightDriveMotor.SetSelectedSensorPosition(0);
    frontLeftDriveMotor.SetSelectedSensorPosition(0);
    backRightDriveMotor.SetSelectedSensorPosition(0);
    backLeftDriveMotor.SetSelectedSensorPosition(0);

    frontLeftAngleMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    frontLeftAngleMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    frontLeftAngleMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    frontLeftAngleMotor.EnableCurrentLimit(true);

    frontRightAngleMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    frontRightAngleMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    frontRightAngleMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    frontRightAngleMotor.EnableCurrentLimit(true);

    backLeftAngleMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    backLeftAngleMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    backLeftAngleMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    backLeftAngleMotor.EnableCurrentLimit(true);

    backRightAngleMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    backRightAngleMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    backRightAngleMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    backRightAngleMotor.EnableCurrentLimit(true);

    frontLeftDriveMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    frontLeftDriveMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    frontLeftDriveMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    frontLeftDriveMotor.EnableCurrentLimit(true);

    frontRightDriveMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    frontRightDriveMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    frontRightDriveMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    frontRightDriveMotor.EnableCurrentLimit(true);

    backLeftDriveMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    backLeftDriveMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    backLeftDriveMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    backLeftDriveMotor.EnableCurrentLimit(true);

    backRightDriveMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    backRightDriveMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    backRightDriveMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    backRightDriveMotor.EnableCurrentLimit(true);
}

void Robot::RobotPeriodic() {}
void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif