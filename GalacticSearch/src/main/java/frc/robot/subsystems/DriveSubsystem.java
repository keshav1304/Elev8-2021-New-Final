// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonSRX FR;
  private final WPI_TalonSRX BR;
  private final SpeedControllerGroup rightSide;

  private final WPI_TalonSRX FL;
  private final WPI_TalonSRX BL;  
  private final SpeedControllerGroup leftSide;

  private final DifferentialDrive driveTrain;

  private double acceleration;
  private double velocity;
  private double displacement; 
  private double previousTime;
  private Timer timer;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // timer = new Timer();
    // resetIntegrals();
    // timer.reset();
    // timer.start();

    FR = new WPI_TalonSRX(Constants.FR_port);
    BR = new WPI_TalonSRX(Constants.BR_port);
    rightSide = new SpeedControllerGroup(FR, BR);
    
    FL = new WPI_TalonSRX(Constants.FL_port);
    BL = new WPI_TalonSRX(Constants.BL_port); 
    leftSide = new SpeedControllerGroup(FL, BL);
    
    driveTrain = new DifferentialDrive(leftSide, rightSide);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateDistance();

    // SmartDashboard.putNumber("Displacement", this.displacement);
    // SmartDashboard.putNumber("Velocity", this.velocity);
    // SmartDashboard.putNumber("Acceleration", this.acceleration);

    // SmartDashboard.putNumber("Enc R", RobotContainer.navx.getYaw());
    // SmartDashboard.putNumber("Enc R", RobotContainer.encR.getDistance() * Constants.rightScale);
    // SmartDashboard.putNumber("Enc L", RobotContainer.encL.getDistance());
    
  }

  public void arcadeInbuilt(double y, double z) {
    FR.setInverted(false);
    BR.setInverted(false);
    driveTrain.arcadeDrive(y * Constants.arcadeMaxSpeed, z * Constants.arcadeMaxSpeed);
  }

  public void brakeMode() {
    FL.setNeutralMode(NeutralMode.Brake);
    FR.setNeutralMode(NeutralMode.Brake);
    BL.setNeutralMode(NeutralMode.Brake);
    BR.setNeutralMode(NeutralMode.Brake);
  }

  public void coastMode() {
    FL.setNeutralMode(NeutralMode.Coast);
    FR.setNeutralMode(NeutralMode.Coast);
    BL.setNeutralMode(NeutralMode.Coast);
    BR.setNeutralMode(NeutralMode.Coast);
  }

  public void drive(double l, double r) {
    FR.setInverted(true);
    BR.setInverted(true);

    FR.set(r);
    BR.set(r);
    FL.set(l);
    BL.set(l);
  }

  public void driveRaw(double y) {
    drive(y * Constants.maxSpeed, y * Constants.maxSpeed);
  }

  public double getAverageDistance() {
    return (RobotContainer.encL.getDistance() + RobotContainer.encR.getDistance())/2;
  }

  public void moveByDistance(double correction) {
    if (Math.abs(correction) < Constants.minSpeed) correction = Math.signum(correction) * Constants.minSpeed;
    if (Math.abs(correction) > Constants.maxSpeed) correction = Math.signum(correction) * Constants.maxSpeed;
    drive(correction, correction);
  }

  public void moveByAngle(double correction) {
    if (Math.abs(correction) < Constants.minSpeed) correction = Math.signum(correction) * Constants.minSpeed;
    if (Math.abs(correction) > Constants.maxSpeed) correction = Math.signum(correction) * Constants.maxSpeed;
    drive(correction, -correction);
  }

  public void alignBall(double angleCorrection, double distanceCorrection) {
    double correctionLeft = /*distanceCorrection*/ + angleCorrection;
    double correctionRight =  /*distanceCorrection*/ - angleCorrection;

    if (Math.abs(correctionLeft) < Constants.minSpeed) correctionLeft = Math.signum(correctionLeft) * Constants.minSpeed;
    if (Math.abs(correctionLeft) > Constants.maxSpeed) correctionLeft = Math.signum(correctionLeft) * Constants.maxSpeed;
    if (Math.abs(correctionRight) < Constants.minSpeed) correctionRight = Math.signum(correctionRight) * Constants.minSpeed;
    if (Math.abs(correctionRight) > Constants.maxSpeed) correctionRight = Math.signum(correctionRight) * Constants.maxSpeed;

    drive(correctionLeft, correctionRight);
  }

  public void swerve(double angleCorrection, double distanceCorrection) {
    double correctionLeft = distanceCorrection + angleCorrection;
    double correctionRight =  distanceCorrection - angleCorrection;

    if (Math.abs(correctionLeft) < Constants.minSpeed) correctionLeft = Math.signum(correctionLeft) * Constants.minSpeed;
    if (Math.abs(correctionLeft) > Constants.maxSpeed) correctionLeft = Math.signum(correctionLeft) * Constants.maxSpeed;
    if (Math.abs(correctionRight) < Constants.minSpeed) correctionRight = Math.signum(correctionRight) * Constants.minSpeed;
    if (Math.abs(correctionRight) > Constants.maxSpeed) correctionRight = Math.signum(correctionRight) * Constants.maxSpeed;

    drive(correctionLeft, correctionRight);
  }

}
