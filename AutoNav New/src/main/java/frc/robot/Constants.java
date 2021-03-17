// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int FR_port = 2;
    public static final int FL_port = 1;
    public static final int BR_port = 4; 
    public static final int BL_port = 3;

    public static double arcadeMaxSpeed = 0.6d;
    public static double maxSpeed = 0.3d;
    public static double minSpeed = 0.1d;
    public static double deadband = 0.02d;

    // Encoders
    public static double encoderScale = 0.001425d;
    public static double rightScale = 0.25d;

    // public static final double T = 20 * Math.pow(10, -3);
    public static final double G = 9.81d;
    public static final double FIELD = 0.762d;

    public static double kPTurn = 0.0085;
    public static double kPDist = 0.21;

    public static final double CAM_WIDTH = 854;
    public static final double MAX_RADIUS = 100;

    public static double navxScale = 1.1d;
    public static double cameraScale = 0.00035d;
    public static double radiusScale = 5.0d;

    public static double ballRadius = 54;
    
}
