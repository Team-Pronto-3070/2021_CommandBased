// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;


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
    public final static int TAL_FL_PORT = 2;
    public final static int TAL_BL_PORT = 1;
    public final static int TAL_FR_PORT = 3;
    public final static int TAL_BR_PORT = 5;

    //Intake Talon Ports
    public final static int TAL_INTAKE_PORT = 4;

    //Intake speed constants
    public static final double IN_SPEED = 1;
    public static final double OUT_SPEED = -0.5;

    //Drive Subsystem Constants
    public static final double RAMP_TIME = 0.4;   //Number of seconds for a falcon to ramp from 0 -> 1
    public static final double INPUT_CAP = 0.95;  //Max input value for a falcon

    public static final double DRIVETRAIN_RADIUS_INCHES = 17.284903; //distance from the center of the robot to the center of the wheels in inches
    public static final double MAX_WHEEL_VELOCITY = 2; //maximum wheel velocity in m/s
    public static final double MAX_ANGULAR_VELOCITY = 4; //in radians/second
    public static final double MAX_ANGULAR_ACCELERATION = 1; //in radians/second^2

    //PID Constants
    public static final PIDController X_PID_CONTROLLER = new PIDController(6, 0.025, 0);
    public static final PIDController Y_PID_CONTROLLER = new PIDController(6, 0.025, 0);
    public static final ProfiledPIDController THETA_PID_CONTROLLER = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION));

    //p = .25
    public static final PIDController VX_PID = new PIDController(0, 0, 0);
    public static final PIDController VY_PID = new PIDController(0, 0, 0);
    public static final PIDController OMEGA_PID = new PIDController(0, 0, 0);

    //p = 750, i = 0
    //p = 600, i = 0.1
    public static final PIDController FL_PID = new PIDController(750, 0, 0);
    public static final PIDController FR_PID = new PIDController(750, 0, 0);
    public static final PIDController BL_PID = new PIDController(750, 0, 0);
    public static final PIDController BR_PID = new PIDController(750, 0, 0);

    public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.845/12.0, 0.25044035865390880768, 0.171/12.0);
    
    public static final SimpleMotorFeedforward FL_FF = new SimpleMotorFeedforward(0.057, 1023.0 * (1.0 - 0.057) / 18000.0, 0);
    public static final SimpleMotorFeedforward FR_FF = new SimpleMotorFeedforward(0.057, 1023.0 * (1.0 - 0.057) / 18000.0, 0);
    public static final SimpleMotorFeedforward BL_FF = new SimpleMotorFeedforward(0.057, 1023.0 * (1.0 - 0.057) / 18000.0, 0);
    public static final SimpleMotorFeedforward BR_FF = new SimpleMotorFeedforward(0.057, 1023.0 * (1.0 - 0.057) / 18000.0, 0);

    public static final double WHEEL_VELOCITY_DEADBAND = 0.1;
    
    //odometry constants
    public static final int[] ODOMETRY_WHEEL_LEFT_PORT = new int[] {14, 15}; //dio pins for odometry wheel encoders
    public static final int[] ODOMETRY_WHEEL_RIGHT_PORT = new int[] {12, 13};
    public static final int[] ODOMETRY_WHEEL_BACK_PORT = new int[] {16, 17};

    public static final boolean ODOMETRY_WHEEL_LEFT_REVERSED = false;
    public static final boolean ODOMETRY_WHEEL_RIGHT_REVERSED = true;
    public static final boolean ODOMETRY_WHEEL_BACK_REVERSED = false;

    public static final double ODOMETRY_WHEEL_METERS_PER_PULSE_L = 0.000155852;
    public static final double ODOMETRY_WHEEL_METERS_PER_PULSE_R = 0.000155852;
    public static final double ODOMETRY_WHEEL_METERS_PER_PULSE_B = 0.000155852;
    
    public static final double ODOMETRY_WHEEL_BACK_INCHES = 13.755204; //distance from the center of the robot to the center of the odometry wheels in inches
    public static final double ODOMETRY_WHEEL_SIDE_INCHES = 12.876524; //left and right radii should be the same

    //autonomous constants
    public static final Pose2d INITIAL_POSE = new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), Rotation2d.fromDegrees(0));

    //Joystick port
    public final static int JOY_PORT = 0;

    //Joystick Deadzone Constants
    public static final double JOY_STICK_VX_DEADZONE = 0.15;
    public static final double JOY_STICK_VY_DEADZONE = 0.15;
    public static final double JOY_STICK_OMEGA_DEADZONE = 0.15;
    
    //Joystick Coefficent Constants
    public static final double VX_COEFFICENT = 3;
    public static final double VY_COEFFICENT = 3;
    public static final double OMEGA_COEFFICENT = 3;

    //Ratio of Ticks per 100 milliseconds (output of getSelectedSensorVelocity) to meters per second
    private static final double WHEEL_GEAR_RATIO = 12.75;
    private static final double TICKS_PER_REVOLUTION = 2048.0;
    private static final double WHEEL_CIRCUMFRENCE = 6.0 * 3.14159 * 0.0254;

    public static final double TICKMS_TO_MSEC = 0.0001833558212343308;
//    public static final double TICKMS_TO_MSEC = 10.0 / TICKS_PER_REVOLUTION / WHEEL_GEAR_RATIO * WHEEL_CIRCUMFRENCE;

    //pose estimator standard deviations
    public static final Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(30));
    public static final Vector<N1> IMU_STD_DEVS = VecBuilder.fill(Units.degreesToRadians(1));
    public static final Vector<N3> ODOMETRY_STD_DEVS = VecBuilder.fill(5, 5, Units.degreesToRadians(50));

    public static final I2C.Port IMU_PORT = I2C.Port.kOnboard;

    //Vision file path
    public static final String PATHS_FILE = "/home/lvuser/PATHS_FILE.txt";
}