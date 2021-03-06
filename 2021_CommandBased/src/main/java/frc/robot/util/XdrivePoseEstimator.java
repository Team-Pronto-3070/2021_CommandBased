// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.estimator.AngleStatistics;
import edu.wpi.first.wpilibj.estimator.UnscentedKalmanFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpiutil.WPIUtilJNI;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;

/**
 * This class wraps an {@link UnscentedKalmanFilter Unscented Kalman Filter} to fuse
 * latency-compensated vision measurements with x-drive encoder velocity measurements. It will
 * correct for noisy measurements and encoder drift. It is intended to be an easy but more accurate
 * drop-in for {@link frc.robot.util.XdriveOdometry}.
 *
 * <p>{@link XdrivePoseEstimator#update} should be called every robot loop. If your loops are
 * faster or slower than the default of 0.02s, then you should change the nominal delta time using
 * the secondary constructor: {@link XdivePoseEstimator#XdrivePoseEstimator(Rotation2d,
 * Pose2d, XdriveKinematics, Matrix, Matrix, Matrix, double)}.
 *
 * <p>{@link XdrivePoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave mostly like regular encoder odometry.
 *
 * <p>Our state-space system is:
 *
 * <p><strong> x = [[x, y, theta]]^T </strong> in the field-coordinate system.
 *
 * <p><strong> u = [[vx, vy, omega]]^T </strong> in the field-coordinate system.
 *
 * <p><strong> y = [[x, y, theta]]^T </strong> in field coords from vision, or <strong> y =
 * [[theta]]^T </strong> from the gyro.
 */
public class XdrivePoseEstimator {
  private final UnscentedKalmanFilter<N3, N3, N1> m_observer;
  private final XdriveKinematics m_kinematics;

  private final double m_nominalDt; // Seconds
  private double m_prevTimeSeconds = -1.0;

  private Rotation2d m_gyroOffset;
  private Rotation2d m_previousAngle;

  private Matrix<N3, N3> m_odometryContR;

  /**
   * Constructs a XdrivePoseEstimator.
   *
   * @param gyroAngle The current gyro angle.
   * @param initialPoseMeters The starting pose estimate.
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param stateStdDevs Standard deviations of model states. Increase these numbers to trust your
   *     model's state estimates less. This matrix is in the form [x, y, theta]^T, with units in
   *     meters and radians.
   * @param localMeasurementStdDevs Standard deviations of the encoder and gyro measurements.
   *     Increase these numbers to trust sensor readings from encoders and gyros less. This matrix
   *     is in the form [theta], with units in radians.
   * @param odometryMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]^T, with units in meters and radians.
   */
  public XdrivePoseEstimator(
      Rotation2d gyroAngle,
      Pose2d initialPoseMeters,
      XdriveKinematics kinematics,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N1, N1> localMeasurementStdDevs,
      Matrix<N3, N1> odometryMeasurementStdDevs) {
    this(
        gyroAngle,
        initialPoseMeters,
        kinematics,
        stateStdDevs,
        localMeasurementStdDevs,
        odometryMeasurementStdDevs,
        0.02);
  }

  /**
   * Constructs a XdrivePoseEstimator.
   *
   * @param gyroAngle The current gyro angle.
   * @param initialPoseMeters The starting pose estimate.
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param stateStdDevs Standard deviations of model states. Increase these numbers to trust your
   *     model's state estimates less. This matrix is in the form [x, y, theta]^T, with units in
   *     meters and radians.
   * @param localMeasurementStdDevs Standard deviations of the encoder and gyro measurements.
   *     Increase these numbers to trust sensor readings from encoders and gyros less. This matrix
   *     is in the form [theta], with units in radians.
   * @param odometryMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]^T, with units in meters and radians.
   * @param nominalDtSeconds The time in seconds between each robot loop.
   */
  @SuppressWarnings("ParameterName")
  public XdrivePoseEstimator(
      Rotation2d gyroAngle,
      Pose2d initialPoseMeters,
      XdriveKinematics kinematics,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N1, N1> localMeasurementStdDevs,
      Matrix<N3, N1> odometryMeasurementStdDevs,
      double nominalDtSeconds) {
    m_nominalDt = nominalDtSeconds;

    m_observer =
        new UnscentedKalmanFilter<>(
            Nat.N3(),
            Nat.N1(),
            (x, u) -> u,
            (x, u) -> x.extractRowVector(2),
            stateStdDevs,
            localMeasurementStdDevs,
            AngleStatistics.angleMean(2),
            AngleStatistics.angleMean(0),
            AngleStatistics.angleResidual(2),
            AngleStatistics.angleResidual(0),
            AngleStatistics.angleAdd(2),
            m_nominalDt);
    m_kinematics = kinematics;

    m_odometryContR = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), odometryMeasurementStdDevs);

    m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = initialPoseMeters.getRotation();
    m_observer.setXhat(StateSpaceUtil.poseTo3dVector(initialPoseMeters));
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>You NEED to reset your encoders (to zero) when calling this method.
   *
   * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param poseMeters The position on the field that your robot is at.
   * @param gyroAngle The angle reported by the gyroscope.
   */
  public void resetPosition(Pose2d poseMeters, Rotation2d gyroAngle) {
    m_previousAngle = poseMeters.getRotation();
    m_gyroOffset = getEstimatedPosition().getRotation().minus(gyroAngle);
    m_observer.reset();
    m_observer.setXhat(StateSpaceUtil.poseTo3dVector(poseMeters));
  }

  /**
   * Gets the pose of the robot at the current time as estimated by the Unscented Kalman Filter.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition() {
    return new Pose2d(
        m_observer.getXhat(0), m_observer.getXhat(1), new Rotation2d(m_observer.getXhat(2)));
  }

  /**
   * Updates the the Unscented Kalman Filter using only wheel encoder information. This should be
   * called every loop, and the correct loop period must be passed into the constructor of this
   * class.
   *
   * @param gyroAngle The current gyro angle.
   * @param wheelSpeeds The current speeds of the x-drive wheels.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d update(Rotation2d gyroAngle, XdriveWheelSpeeds wheelSpeeds, Pose2d odometryMeasurement) {
    return updateWithTime(WPIUtilJNI.now() * 1.0e-6, gyroAngle, wheelSpeeds, odometryMeasurement);
  }

  /**
   * Updates the the Unscented Kalman Filter using only wheel encoder information. This should be
   * called every loop, and the correct loop period must be passed into the constructor of this
   * class.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle The current gyroscope angle.
   * @param wheelSpeeds The current speeds of the x-drive wheels.
   * @return The estimated pose of the robot in meters.
   */
  @SuppressWarnings("LocalVariableName")
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, XdriveWheelSpeeds wheelSpeeds, Pose2d odometryMeasurement) {
    double dt = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDt;
    m_prevTimeSeconds = currentTimeSeconds;

    var angle = gyroAngle.plus(m_gyroOffset);
    var omega = angle.minus(m_previousAngle).getRadians() / dt;

    var chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);
    var fieldRelativeVelocities =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(angle);

    var u = VecBuilder.fill(fieldRelativeVelocities.getX(), fieldRelativeVelocities.getY(), omega);
    m_previousAngle = angle;

    var localY = VecBuilder.fill(angle.getRadians());
    m_observer.predict(u, dt);
    m_observer.correct(u, localY);
    m_observer.correct(
      Nat.N3(),
      u,
      StateSpaceUtil.poseTo3dVector(odometryMeasurement),
      (xv, uv) -> xv,
      m_odometryContR,
      AngleStatistics.angleMean(2),
      AngleStatistics.angleResidual(2),
      AngleStatistics.angleResidual(2),
      AngleStatistics.angleAdd(2));

    return getEstimatedPosition();
  }
}
