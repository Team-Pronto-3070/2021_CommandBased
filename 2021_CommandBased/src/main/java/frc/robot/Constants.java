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

    //Drive Talon Ports
    public final static int TAL_FL_PORT = -1;
    public final static int TAL_BL_PORT = -1;
    public final static int TAL_FR_PORT = -1;
    public final static int TAL_BR_PORT = -1;

    //Intake Talon Ports
    public final static int TAL_INTAKE_PORT = -1;

    //Drive Subsystem Constants
    public static double RAMP_TIME = 0.4;   //Number of seconds for a falcon to ramp from 0 -> 1
    public static double INPUT_CAP = 0.95;  //Max input value for a falcon
    public static final double DRIVETRAIN_RADIUS_INCHES = 17.284903; //distance from the center of the robot to the center of the wheels in inches
    public static final double MAX_WHEEL_VELOCITY = 20; //maximum wheel velocity in m/s

    //odometry constants
    public static final int[] ODOMETRY_WHEEL_LEFT_PORT = new int[] {-1, -1}; //dio pins for odometry wheel encoders
    public static final int[] ODOMETRY_WHEEL_RIGHT_PORT = new int[] {-1, -1};
    public static final int[] ODOMETRY_WHEEL_BACK_PORT = new int[] {-1, -1};

    public static final boolean ODOMETRY_WHEEL_LEFT_REVERSED = false;
    public static final boolean ODOMETRY_WHEEL_RIGHT_REVERSED = false;
    public static final boolean ODOMETRY_WHEEL_BACK_REVERSED = false;

    public static final double ODOMETRY_WHEEL_METERS_PER_PULSE = 1;

    public static final double ODOMETRY_WHEEL_BACK_INCHES = 13.755204; //distance from the center of the robot to the center of the odometry wheels in inches
    public static final double ODOMETRY_WHEEL_SIDE_INCHES = 12.876524; //left and right radii should be the same

    //Joystick port
    public final static int JOY_PORT = -1;

    //Intake speed
    public final static double INTAKE_SPEED = 0.0;
}