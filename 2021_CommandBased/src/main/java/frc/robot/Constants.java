// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;


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

    //Intake speed constants
    public static final double IN_SPEED = 0.5;
    public static final double OUT_SPEED = -0.5;

    //Drive Subsystem Constants
    public static final double RAMP_TIME = 0.4;   //Number of seconds for a falcon to ramp from 0 -> 1
    public static final double INPUT_CAP = 0.95;  //Max input value for a falcon

    public static final double DRIVETRAIN_RADIUS_INCHES = 17.284903; //distance from the center of the robot to the center of the wheels in inches
    public static final double MAX_WHEEL_VELOCITY = 5; //maximum wheel velocity in m/s
    public static final double MAX_ANGULAR_VELOCITY = 10; //in radians/second
    public static final double MAX_ANGULAR_ACCELERATION = 3; //in radians/second^2

    //PID Constants
    public static final PIDController X_PID_CONTROLLER = new PIDController(1, 0, 0);
    public static final PIDController Y_PID_CONTROLLER = new PIDController(1, 0, 0);
    public static final ProfiledPIDController THETA_PID_CONTROLLER = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION));

    public static final PIDController FL_PID = new PIDController(1, 0, 0);
    public static final PIDController FR_PID = new PIDController(1, 0, 0);
    public static final PIDController BL_PID = new PIDController(1, 0, 0);
    public static final PIDController BR_PID = new PIDController(1, 0, 0);
    
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

    //autonomous constants
    public static final Pose2d INITIAL_POSE = new Pose2d(0, 0, new Rotation2d(0));

    //Joystick port
    public final static int JOY_PORT = -1;

    //Joystick Deadzone Constants
    public static final double JOY_STICK_VX_DEADZONE = 0.1;
    public static final double JOY_STICK_VY_DEADZONE = 0.1;
    public static final double JOY_STICK_OMEGA_DEADZONE = 0.1;
    
    //Joystick Coefficent Constants
    public static final double VX_COEFFICENT = 5;
    public static final double VY_COEFFICENT = 5;
    public static final double OMEGA_COEFFICENT = 5;
}