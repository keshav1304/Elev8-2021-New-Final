// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class BallFollowingCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  double angleError, distanceError;

  /** Creates a new BallFollowingCommand. */
  public BallFollowingCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xCenter = SmartDashboard.getNumber("CenterX", 0.0d);
    double radius = SmartDashboard.getNumber("Radius", Constants.MAX_RADIUS);
    this.angleError = ((Constants.CAM_WIDTH * 0.5d) - xCenter) * -1 * Constants.cameraScale;
    this.distanceError = (Constants.MAX_RADIUS - radius) * Constants.cameraScale * Constants.radiusScale;
    this.driveSubsystem.followBall(angleError, distanceError);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (!RobotContainer.joy1.getRawButton(5) || (this.angleError <= Constants.deadband && this.distanceError <= Constants.MAX_RADIUS * Constants.deadband * Constants.radiusScale));
    return (Math.abs(this.angleError) <= Constants.deadband && Math.abs(this.distanceError) <= Constants.deadband);
  }
}
