// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import frc.robot.Constants;

/**
 * Class for x-drive odometry. Odometry allows you to track the robot's position on the field
 * over a course of a match using readings from your odometry wheels.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like path following.
 * Furthermore, odometry can be used for latency compensation when using computer-vision systems.
 */
public class XdriveOdometry {

  private Pose2d initialPose;

  private Encoder leftEncoder;
  private Encoder rightEncoder;
  private Encoder backEncoder;

  private final SimpleMatrix forwardKinematics;
  private final SimpleMatrix inverseKinematics;

  /**
   * Constructs a XdriveOdometry object.
   * 
   * @param initialPoseMeters The starting position of the robot on the field.
   */
  public XdriveOdometry(Pose2d initialPoseMeters) {
    initialPose = initialPoseMeters;
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

    forwardKinematics = new SimpleMatrix(new double[][] {{ 1, 0, -1 * Units.inchesToMeters(Constants.ODOMETRY_WHEEL_SIDE_INCHES)},
                                                         { 1, 0,      Units.inchesToMeters(Constants.ODOMETRY_WHEEL_SIDE_INCHES)},
                                                         { 0, 1, -1 * Units.inchesToMeters(Constants.ODOMETRY_WHEEL_BACK_INCHES)}});
    inverseKinematics = forwardKinematics.invert();
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
    initialPose = poseMeters;
    leftEncoder.reset();
    rightEncoder.reset();
    backEncoder.reset();
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose2d getPoseMeters() {
    var poseVector = inverseKinematics.mult(new SimpleMatrix(new double[][] {{leftEncoder.getDistance() },
                                                                             {rightEncoder.getDistance()},
                                                                             {backEncoder.getDistance() }}));
    return new Pose2d(new Translation2d(poseVector.get(0,0), poseVector.get(1,0))
                            .rotateBy(new Rotation2d(poseVector.get(2,0))),
                      new Rotation2d(poseVector.get(2,0)))
                .relativeTo(initialPose);
  }

  /*
  public Pose2d getPoseWith2Encoders(Rotation2d gyroAngle) {
    rightDist = rightEncoder.getDistance();
    backDist = backEncoder.getDistance();

    SmartDashboard.putNumber("right_encoder", rightDist);
    SmartDashboard.putNumber("back_encoder", backDist);
 
    x = rightDist - (gyroAngle.getRadians() * Units.inchesToMeters(Constants.ODOMETRY_WHEEL_SIDE_INCHES));
    y = backDist + (Units.inchesToMeters(Constants.ODOMETRY_WHEEL_BACK_INCHES) * gyroAngle.getRadians());
    
    return new Pose2d(x, y, gyroAngle).plus(new Transform2d(new Pose2d(), initialPose));
  }
  */

  /**
   * 
   */
  public Pose2d update() {
    SmartDashboard.putNumber("left_encoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("right_encoder", rightEncoder.getDistance());
    SmartDashboard.putNumber("back_encoder", backEncoder.getDistance());
    return getPoseMeters();
  }
}