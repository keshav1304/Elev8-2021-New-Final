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
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MoveByDistanceCommand;
import frc.robot.commands.ShooterCommand;
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
  //private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // Commands
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  // private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem);
  // IO Devices
  public static Joystick joy1 = new Joystick(1);

  public static Encoder encR = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  public static Encoder encL = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
  public static AHRS navx = new AHRS(SPI.Port.kMXP);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // shooterSubsystem.setDefaultCommand(shooterCommand);
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

// //   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    List<double[]> coordinates = new ArrayList<double[]>();

    coordinates.add(new double[]{0d, 4.5});
    coordinates.add(new double[]{1.75d, 4.5d});
    coordinates.add(new double[]{1.5d, 1d});
    coordinates.add(new double[]{1d, 1d});
    coordinates.add(new double[]{1.2d, 2.5d});

    
   

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
