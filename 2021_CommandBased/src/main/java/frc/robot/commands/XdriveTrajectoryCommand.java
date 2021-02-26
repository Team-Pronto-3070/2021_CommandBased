// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.XdriveKinematics;
import frc.robot.util.XdriveMotorVoltages;
import frc.robot.util.XdriveWheelSpeeds;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a x-drive.
 *
 * <p>The command handles trajectory-following, Velocity PID calculations, and feedforwards
 * internally. This is intended to be a more-or-less "complete solution" that can be used by teams
 * without a great deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard PID
 * functionality of a "smart" motor controller) may use the secondary constructor that omits the PID
 * and feedforward functionality, returning only the raw wheel speeds from the PID controllers.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 */
public class XdriveTrajectoryCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private final boolean m_usePID;
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SimpleMotorFeedforward m_feedforward;
  private final XdriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Supplier<Rotation2d> m_desiredRotation;
  private final double m_maxWheelVelocityMetersPerSecond;
  private final PIDController m_frontLeftController;
  private final PIDController m_rearLeftController;
  private final PIDController m_frontRightController;
  private final PIDController m_rearRightController;
  private final Supplier<XdriveWheelSpeeds> m_currentWheelSpeeds;
  private final Consumer<XdriveMotorVoltages> m_outputDriveVoltages;
  private final Consumer<XdriveWheelSpeeds> m_outputWheelSpeeds;
  private XdriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  /**
   * Constructs a new XdriveControllerCommand that when executed will follow the provided
   * trajectory. PID control and feedforward are handled internally. Outputs are scaled from -12 to
   * 12 as a voltage output to the motor.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param feedforward The feedforward to use for the drivetrain.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the robot should be facing. This is sampled at each time
   *     step.
   * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
   * @param frontLeftController The front left wheel velocity PID.
   * @param rearLeftController The rear left wheel velocity PID.
   * @param frontRightController The front right wheel velocity PID.
   * @param rearRightController The rear right wheel velocity PID.
   * @param currentWheelSpeeds A XdriveWheelSpeeds object containing the current wheel speeds.
   * @param outputDriveVoltages A XdriveMotorVoltages object containing the output motor
   *     voltages.
   * @param requirements The subsystems to require.
   */
  public XdriveTrajectoryCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SimpleMotorFeedforward feedforward,
      XdriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      double maxWheelVelocityMetersPerSecond,
      PIDController frontLeftController,
      PIDController rearLeftController,
      PIDController frontRightController,
      PIDController rearRightController,
      Supplier<XdriveWheelSpeeds> currentWheelSpeeds,
      Consumer<XdriveMotorVoltages> outputDriveVoltages,
      Subsystem... requirements) {
    m_trajectory = trajectory;
    m_pose = pose;
    m_feedforward = feedforward;
    m_kinematics = kinematics;

    m_controller = new HolonomicDriveController(xController, yController, thetaController);

    m_desiredRotation = desiredRotation;

    m_maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;

    m_frontLeftController = frontLeftController;
    m_rearLeftController = rearLeftController;
    m_frontRightController = frontRightController;
    m_rearRightController = rearRightController;

    m_currentWheelSpeeds = currentWheelSpeeds;

    m_outputDriveVoltages = outputDriveVoltages;

    m_outputWheelSpeeds = null;

    m_usePID = true;

    addRequirements(requirements);
  }

  /**
   * Constructs a new XdriveControllerCommand that when executed will follow the provided
   * trajectory. PID control and feedforward are handled internally. Outputs are scaled from -12 to
   * 12 as a voltage output to the motor.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param feedforward The feedforward to use for the drivetrain.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
   * @param frontLeftController The front left wheel velocity PID.
   * @param rearLeftController The rear left wheel velocity PID.
   * @param frontRightController The front right wheel velocity PID.
   * @param rearRightController The rear right wheel velocity PID.
   * @param currentWheelSpeeds A XdriveWheelSpeeds object containing the current wheel speeds.
   * @param outputDriveVoltages A XdriveMotorVoltages object containing the output motor
   *     voltages.
   * @param requirements The subsystems to require.
   */
  public XdriveTrajectoryCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SimpleMotorFeedforward feedforward,
      XdriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      double maxWheelVelocityMetersPerSecond,
      PIDController frontLeftController,
      PIDController rearLeftController,
      PIDController frontRightController,
      PIDController rearRightController,
      Supplier<XdriveWheelSpeeds> currentWheelSpeeds,
      Consumer<XdriveMotorVoltages> outputDriveVoltages,
      Subsystem... requirements) {

    this(
        trajectory,
        pose,
        feedforward,
        kinematics,
        xController,
        yController,
        thetaController,
        () ->
            trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        maxWheelVelocityMetersPerSecond,
        frontLeftController,
        rearLeftController,
        frontRightController,
        rearRightController,
        currentWheelSpeeds,
        outputDriveVoltages,
        requirements);
  }

  /**
   * Constructs a new XdriveControllerCommand that when executed will follow the provided
   * trajectory. The user should implement a velocity PID on the desired output wheel velocities.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with non-stationary end-states.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the robot should be facing. This is sampled at each time
   *     step.
   * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
   * @param outputWheelSpeeds A XdriveWheelSpeeds object containing the output wheel speeds.
   * @param requirements The subsystems to require.
   */
  public XdriveTrajectoryCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      XdriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      double maxWheelVelocityMetersPerSecond,
      Consumer<XdriveWheelSpeeds> outputWheelSpeeds,
      Subsystem... requirements) {
    m_trajectory = trajectory;
    m_pose = pose;
    m_feedforward = new SimpleMotorFeedforward(0, 0, 0);
    m_kinematics = kinematics;

    m_controller = new HolonomicDriveController(xController, yController, thetaController);

    m_desiredRotation = desiredRotation;

    m_maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;

    m_frontLeftController = null;
    m_rearLeftController = null;
    m_frontRightController = null;
    m_rearRightController = null;

    m_currentWheelSpeeds = null;

    m_outputWheelSpeeds = outputWheelSpeeds;

    m_outputDriveVoltages = null;

    m_usePID = false;

    addRequirements(requirements);
  }

  /**
   * Constructs a new XdriveControllerCommand that when executed will follow the provided
   * trajectory. The user should implement a velocity PID on the desired output wheel velocities.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with non-stationary end-states.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
   * @param outputWheelSpeeds A XdriveWheelSpeeds object containing the output wheel speeds.
   * @param requirements The subsystems to require.
   */
  public XdriveTrajectoryCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      XdriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      double maxWheelVelocityMetersPerSecond,
      Consumer<XdriveWheelSpeeds> outputWheelSpeeds,
      Subsystem... requirements) {
    this(
        trajectory,
        pose,
        kinematics,
        xController,
        yController,
        thetaController,
        () ->
            trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        maxWheelVelocityMetersPerSecond,
        outputWheelSpeeds,
        requirements);
  }

  @Override
  public void initialize() {
    var initialState = m_trajectory.sample(0);

    var initialXVelocity =
        initialState.velocityMetersPerSecond * initialState.poseMeters.getRotation().getCos();
    var initialYVelocity =
        initialState.velocityMetersPerSecond * initialState.poseMeters.getRotation().getSin();

    m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialXVelocity, initialYVelocity, 0.0));

    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    var desiredState = m_trajectory.sample(curTime);

    var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(targetChassisSpeeds);

    targetWheelSpeeds.normalize(m_maxWheelVelocityMetersPerSecond);

    var frontLeftSpeedSetpoint = targetWheelSpeeds.frontLeftMetersPerSecond;
    var rearLeftSpeedSetpoint = targetWheelSpeeds.rearLeftMetersPerSecond;
    var frontRightSpeedSetpoint = targetWheelSpeeds.frontRightMetersPerSecond;
    var rearRightSpeedSetpoint = targetWheelSpeeds.rearRightMetersPerSecond;

    double frontLeftOutput;
    double rearLeftOutput;
    double frontRightOutput;
    double rearRightOutput;

    if (m_usePID) {
      final double frontLeftFeedforward =
          m_feedforward.calculate(
              frontLeftSpeedSetpoint,
              (frontLeftSpeedSetpoint - m_prevSpeeds.frontLeftMetersPerSecond) / dt);

      final double rearLeftFeedforward =
          m_feedforward.calculate(
              rearLeftSpeedSetpoint,
              (rearLeftSpeedSetpoint - m_prevSpeeds.rearLeftMetersPerSecond) / dt);

      final double frontRightFeedforward =
          m_feedforward.calculate(
              frontRightSpeedSetpoint,
              (frontRightSpeedSetpoint - m_prevSpeeds.frontRightMetersPerSecond) / dt);

      final double rearRightFeedforward =
          m_feedforward.calculate(
              rearRightSpeedSetpoint,
              (rearRightSpeedSetpoint - m_prevSpeeds.rearRightMetersPerSecond) / dt);

      frontLeftOutput =
          frontLeftFeedforward
              + m_frontLeftController.calculate(
                  m_currentWheelSpeeds.get().frontLeftMetersPerSecond, frontLeftSpeedSetpoint);

      rearLeftOutput =
          rearLeftFeedforward
              + m_rearLeftController.calculate(
                  m_currentWheelSpeeds.get().rearLeftMetersPerSecond, rearLeftSpeedSetpoint);

      frontRightOutput =
          frontRightFeedforward
              + m_frontRightController.calculate(
                  m_currentWheelSpeeds.get().frontRightMetersPerSecond, frontRightSpeedSetpoint);

      rearRightOutput =
          rearRightFeedforward
              + m_rearRightController.calculate(
                  m_currentWheelSpeeds.get().rearRightMetersPerSecond, rearRightSpeedSetpoint);

      m_outputDriveVoltages.accept(
          new XdriveMotorVoltages(
              frontLeftOutput, frontRightOutput, rearLeftOutput, rearRightOutput));

    } else {
      m_outputWheelSpeeds.accept(
          new XdriveWheelSpeeds(
              frontLeftSpeedSetpoint,
              frontRightSpeedSetpoint,
              rearLeftSpeedSetpoint,
              rearRightSpeedSetpoint));
    }

    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}