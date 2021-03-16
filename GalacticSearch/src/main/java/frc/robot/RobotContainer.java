// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import java.lang.*;
import java.util.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MoveByDistanceCommand;
import frc.robot.commands.MoveByAngleCommand;
import frc.robot.commands.BallFollowingCommand;
import frc.robot.commands.CoordinateFollowingCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // Commands
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);

  // IO Devices
  public static Joystick joy1 = new Joystick(1);

  public static Encoder encR = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  public static Encoder encL = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
  public static AHRS navx = new AHRS(SPI.Port.kMXP);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(driveCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    List<double[]> coordinates;

    // Path A

    if (SmartDashboard.getNumber("Radius", 1) >= Constants.ballRadius) {
      coordinates = new ArrayList<double[]>();
      coordinates.add(new double[]{0d, 2.8d});
      coordinates.add(new double[]{1.3d, 4.8d});
      coordinates.add(new double[]{-2d, 5.8d});
      coordinates.add(new double[]{0d, 11d});
    }
    else {
      coordinates = new ArrayList<double[]>();
      coordinates.add(new double[]{0d, 3d});
      coordinates.add(new double[]{2.27d, 5.7d});
      coordinates.add(new double[]{-1.1d, 7d});
      coordinates.add(new double[]{0.5d, 9d});
      coordinates.add(new double[]{0d, 11d});
    }


    // Path B

    // if (SmartDashboard.getNumber("Radius", 1) >= Constants.ballRadius) {
    //   coordinates = new ArrayList<double[]>();
    //   coordinates.add(new double[]{0d, 2.6d});
    //   coordinates.add(new double[]{2.4d, 5d});
    //   coordinates.add(new double[]{0d, 6.7d});
    //   coordinates.add(new double[]{0.4d, 11d});
    // }
    // else {
    //   coordinates = new ArrayList<double[]>();
    //   coordinates.add(new double[]{2.25d, 6d});
    //   coordinates.add(new double[]{-0.3d, 8d});
    //   coordinates.add(new double[]{2.2d, 10d});
    //   coordinates.add(new double[]{2d, 11d});
    // }

    // AR
    // List<double[]> coordinates = new ArrayList<double[]>();
    // coordinates.add(new double[]{0d, 2.8d});
    // coordinates.add(new double[]{1.3d, 4.8d});
    // coordinates.add(new double[]{-2d, 5.8d});
    // coordinates.add(new double[]{0d, 11d});

    // AB
    // List<double[]> coordinates = new ArrayList<double[]>();
    // coordinates.add(new double[]{0d, 3d});
    // coordinates.add(new double[]{2.27d, 5.7d});
    // coordinates.add(new double[]{-1.1d, 7d});
    // coordinates.add(new double[]{0.5d, 9d});
    // coordinates.add(new double[]{0d, 11d});

    // BR, B = 0, start the robot on B1
    // List<double[]> coordinates = new ArrayList<double[]>();
    // coordinates.add(new double[]{0d, 2.6d});
    // coordinates.add(new double[]{2.4d, 5d});
    // coordinates.add(new double[]{0d, 67d});
    // coordinates.add(new double[]{0.4d, 11d});

    // BB, B = 0, start the robot on B1
    // List<double[]> coordinates = new ArrayList<double[]>();
    // coordinates.add(new double[]{0d, 1.5d});
    // coordinates.add(new double[]{2.25d, 6d});
    // coordinates.add(new double[]{-0.3d, 8d});
    // coordinates.add(new double[]{2.35d, 10d});
    // coordinates.add(new double[]{2d, 11d});


    return new CoordinateFollowingCommand(this.driveSubsystem, coordinates);
  }

  public static double getY(Joystick joy, double deadband) {
    double value = -1 * joy.getY();
    if (Math.abs(value) < deadband) return 0;
    return value;
  }

  public static double getZ(Joystick joy, double deadband) {
    double value = joy.getZ();
    if (Math.abs(value) < deadband) return 0;
    return value;
  }

}
