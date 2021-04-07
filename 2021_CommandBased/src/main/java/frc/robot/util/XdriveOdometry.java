// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Class for x-drive odometry. Odometry allows you to track the robot's position on the field
 * over a course of a match using readings from your odometry wheels.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like path following.
 * Furthermore, odometry can be used for latency compensation when using computer-vision systems.
 */
public class XdriveOdometry {

  private Pose2d pose;

  private Encoder leftEncoder;
  private Encoder rightEncoder;
  private Encoder backEncoder;

  private final Matrix<N3, N3> inverseKinematics;

  private Vector<N3> prevEncoders = new Vector<>(Nat.N3());

  /**
   * Constructs a XdriveOdometry object.
   * 
   * @param initialPoseMeters The starting position of the robot on the field.
   */
  public XdriveOdometry(Pose2d initialPoseMeters) {
    pose = initialPoseMeters;

    leftEncoder = new Encoder(Constants.ODOMETRY_WHEEL_LEFT_PORT[0],
                              Constants.ODOMETRY_WHEEL_LEFT_PORT[1],
                              Constants.ODOMETRY_WHEEL_LEFT_REVERSED);
    
    rightEncoder = new Encoder(Constants.ODOMETRY_WHEEL_RIGHT_PORT[0],
                               Constants.ODOMETRY_WHEEL_RIGHT_PORT[1],
                               Constants.ODOMETRY_WHEEL_RIGHT_REVERSED);
    
    backEncoder = new Encoder(Constants.ODOMETRY_WHEEL_BACK_PORT[0],
                              Constants.ODOMETRY_WHEEL_BACK_PORT[1],
                              Constants.ODOMETRY_WHEEL_BACK_REVERSED);

    leftEncoder.setDistancePerPulse(Constants.ODOMETRY_WHEEL_METERS_PER_PULSE_L);
    rightEncoder.setDistancePerPulse(Constants.ODOMETRY_WHEEL_METERS_PER_PULSE_R);
    backEncoder.setDistancePerPulse(Constants.ODOMETRY_WHEEL_METERS_PER_PULSE_B);

    inverseKinematics = new MatBuilder<>(Nat.N3(), Nat.N3()).fill(1, 0, -1 * Units.inchesToMeters(Constants.ODOMETRY_WHEEL_SIDE_INCHES),
                                                                  1, 0,      Units.inchesToMeters(Constants.ODOMETRY_WHEEL_SIDE_INCHES),
                                                                  0, 1, -1 * Units.inchesToMeters(Constants.ODOMETRY_WHEEL_BACK_INCHES)).inv();
  }

  /**
   * Constructs a XdriveOdometry object with the default pose at the origin.
   */
  public XdriveOdometry() {
    this(new Pose2d());
  }

  /**
   * Resets the robot's position on the field.
   *
   * @param poseMeters The position on the field that your robot is at.
   */
  public void resetPosition(Pose2d poseMeters) {
    pose = poseMeters;
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose2d getPoseMeters() {
    return pose;
  }

  public ChassisSpeeds getChassisSpeeds() {
    var encoderSpeeds = VecBuilder.fill(leftEncoder.getRate(), rightEncoder.getRate(), backEncoder.getRate());
    var chassisSpeed = inverseKinematics.times(encoderSpeeds);
    return new ChassisSpeeds(chassisSpeed.get(0,0), chassisSpeed.get(1,0), chassisSpeed.get(2,0));
  }

  public Pose2d update() {
    SmartDashboard.putNumber("left_encoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("right_encoder", rightEncoder.getDistance());
    SmartDashboard.putNumber("back_encoder", backEncoder.getDistance());

    var encoders = VecBuilder.fill(leftEncoder.getDistance(), rightEncoder.getDistance(), backEncoder.getDistance());
    var dPose = inverseKinematics.times(encoders.minus(prevEncoders));
    prevEncoders = encoders;
    pose = pose.exp(new Twist2d(dPose.get(0,0), dPose.get(1,0), dPose.get(2,0)));
    return pose;
  }

  public Pose2d updateWithGyro(Rotation2d gyro) {
    pose = pose.exp(new Twist2d(leftEncoder.getDistance() + (Units.inchesToMeters(Constants.ODOMETRY_WHEEL_SIDE_INCHES) * gyro.getRadians()),
                                backEncoder.getDistance() + (Units.inchesToMeters(Constants.ODOMETRY_WHEEL_BACK_INCHES) * gyro.getRadians()),
                                gyro.getRadians()));
    return pose;
  }
}