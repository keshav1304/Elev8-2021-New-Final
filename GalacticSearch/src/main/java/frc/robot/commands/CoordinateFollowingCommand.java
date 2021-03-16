// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.*;
import java.util.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveByDistanceCommand;
import frc.robot.commands.BallFollowingCommand;
import frc.robot.commands.MoveByAngleCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoordinateFollowingCommand extends SequentialCommandGroup {
  /** Creates a new CoordinateFollowingCommand. */
  public CoordinateFollowingCommand(DriveSubsystem driveSubsystem, List<double[]> coordinates) {

    // List<Command> commandList = new ArrayList<Command>();
    double prevX = 0;
    double prevY = 0;
    double prevAngle = 0;

    for (int i=0; i<coordinates.size(); i++) {

      double x = coordinates.get(i)[0];
      double y = coordinates.get(i)[1];

      double deltaX = x - prevX;
      double deltaY = y - prevY;

      double slopeReciprocal = (deltaX / deltaY);

      double desiredAngle = Math.toDegrees(Math.atan(slopeReciprocal)) - prevAngle;
      double desiredDistance = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY)) * Constants.FIELD;

      SmartDashboard.putNumber("Angle " + i, desiredAngle);
      SmartDashboard.putNumber("Distance " + i, desiredDistance);

      addCommands(new MoveByAngleCommand(driveSubsystem, desiredAngle));
      addCommands(new BallFollowingCommand(driveSubsystem));
      addCommands(new MoveByDistanceCommand(driveSubsystem, Constants.FIELD));
      //addCommands(new MoveByDistanceCommand(driveSubsystem, desiredDistance));

      prevX = x;
      prevY = y;
      prevAngle += desiredAngle;

    }
  }
}
