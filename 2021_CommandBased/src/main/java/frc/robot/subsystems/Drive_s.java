package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BiFunction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.commands.XdriveTrajectoryCommand;
import frc.robot.util.XdriveKinematics;
import frc.robot.util.XdriveOdometry;
import frc.robot.util.XdrivePoseEstimator;
import frc.robot.util.XdriveWheelSpeeds;

public class Drive_s extends SubsystemBase{

    //create motor objects
    TalonFX talFL;
    TalonFX talFR;
    TalonFX talBL;
    TalonFX talBR;

    //create kinematics
    XdriveKinematics kinematics;

    //create odometry
    XdriveOdometry odometry;

    private final XdrivePoseEstimator poseEstimator;

    private AHRS imu;

    private Field2d field = new Field2d();

    private Rotation2d gyroOffset = new Rotation2d();
    
    public Drive_s(){
        talFL = new TalonFX(Constants.TAL_FL_PORT);
        talBL = new TalonFX(Constants.TAL_BL_PORT);
        talFR = new TalonFX(Constants.TAL_FR_PORT);
        talBR = new TalonFX(Constants.TAL_BR_PORT);

        talFL.configFactoryDefault();
        talFR.configFactoryDefault();
        talBL.configFactoryDefault();
        talBR.configFactoryDefault();

        talFL.setInverted(InvertType.InvertMotorOutput);
        talFR.setInverted(InvertType.InvertMotorOutput);
        talBL.setInverted(InvertType.InvertMotorOutput);
        talBR.setInverted(InvertType.InvertMotorOutput);

        talFL.setNeutralMode(NeutralMode.Coast);
        talFR.setNeutralMode(NeutralMode.Coast);
        talBL.setNeutralMode(NeutralMode.Coast);
        talBR.setNeutralMode(NeutralMode.Coast);

        talFL.configOpenloopRamp(Constants.RAMP_TIME);
        talBL.configOpenloopRamp(Constants.RAMP_TIME);
        talFR.configOpenloopRamp(Constants.RAMP_TIME);
        talBR.configOpenloopRamp(Constants.RAMP_TIME);

        talFL.configClosedloopRamp(Constants.RAMP_TIME);
        talFR.configClosedloopRamp(Constants.RAMP_TIME);
        talBL.configClosedloopRamp(Constants.RAMP_TIME);
        talBR.configClosedloopRamp(Constants.RAMP_TIME);

        talFL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		talFR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		talBL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        talBR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        talFL.setSensorPhase(true);
        talFR.setSensorPhase(true);
        talBL.setSensorPhase(true);
        talBR.setSensorPhase(true);

        talFL.configSelectedFeedbackCoefficient(1);
        talFR.configSelectedFeedbackCoefficient(1);
        talBL.configSelectedFeedbackCoefficient(1);
        talBR.configSelectedFeedbackCoefficient(1);

        talFL.config_kP(0, Constants.FL_PID.getP() * Constants.TICKMS_TO_MSEC);
        talFL.config_kI(0, Constants.FL_PID.getI() * Constants.TICKMS_TO_MSEC);
        talFL.config_kD(0, Constants.FL_PID.getD() * Constants.TICKMS_TO_MSEC);
        talFL.config_kF(0, Constants.FL_FF.kv);
        
        talFR.config_kP(0, Constants.FR_PID.getP() * Constants.TICKMS_TO_MSEC);
        talFR.config_kI(0, Constants.FR_PID.getI() * Constants.TICKMS_TO_MSEC);
        talFR.config_kD(0, Constants.FR_PID.getD() * Constants.TICKMS_TO_MSEC);
        talFR.config_kF(0, Constants.FR_FF.kv);

        talBL.config_kP(0, Constants.BL_PID.getP() * Constants.TICKMS_TO_MSEC);
        talBL.config_kI(0, Constants.BL_PID.getI() * Constants.TICKMS_TO_MSEC);
        talBL.config_kD(0, Constants.BL_PID.getD() * Constants.TICKMS_TO_MSEC);
        talBL.config_kF(0, Constants.BL_FF.kv);

        talBR.config_kP(0, Constants.BR_PID.getP() * Constants.TICKMS_TO_MSEC);
        talBR.config_kI(0, Constants.BR_PID.getI() * Constants.TICKMS_TO_MSEC);
        talBR.config_kD(0, Constants.BR_PID.getD() * Constants.TICKMS_TO_MSEC);
        talBR.config_kF(0, Constants.BR_FF.kv);

        imu = new AHRS(Constants.IMU_PORT);

        //initialize kinematics with relative locations of wheels
        kinematics = new XdriveKinematics(new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(3 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(1 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(5 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(7 * Math.PI / 4)));
        
        //initialize odometry
        odometry = new XdriveOdometry();

        poseEstimator = new XdrivePoseEstimator(imu.getRotation2d(),
                                                Constants.INITIAL_POSE,
                                                kinematics,
                                                Constants.STATE_STD_DEVS,
                                                Constants.IMU_STD_DEVS,
                                                Constants.ODOMETRY_STD_DEVS);
        
        SmartDashboard.putData("Field", field);
    }

    /**
     * Sets each motor speed separately. 
     * 
     * @param inputValues Double array for speeds of motors between -1 and 1 | [FL, FR, BL, BR]
     */
    public void setIndividual(double[] inputValues){
        talFL.set(ControlMode.PercentOutput, inputValues[0]);
        talFR.set(ControlMode.PercentOutput, inputValues[1]);
        talBL.set(ControlMode.PercentOutput, inputValues[2]);
        talBR.set(ControlMode.PercentOutput, inputValues[3]);
    }

    public void setIndividual(XdriveWheelSpeeds values) {
        setIndividual(new double[] {values.frontLeftMetersPerSecond,
                                    values.frontRightMetersPerSecond,
                                    values.rearLeftMetersPerSecond,
                                    values.rearRightMetersPerSecond});
    }

    public void setWheelSpeeds(XdriveWheelSpeeds wheelSpeeds) {
        SmartDashboard.putNumber("FL_SETPOINT", wheelSpeeds.frontLeftMetersPerSecond);
        SmartDashboard.putNumber("FR_SETPOINT", wheelSpeeds.frontRightMetersPerSecond);
        SmartDashboard.putNumber("BL_SETPOINT", wheelSpeeds.rearLeftMetersPerSecond);
        SmartDashboard.putNumber("BR_SETPOINT", wheelSpeeds.rearRightMetersPerSecond);

        talFL.set(ControlMode.Velocity, Math.abs(wheelSpeeds.frontLeftMetersPerSecond ) > Constants.WHEEL_VELOCITY_DEADBAND ? wheelSpeeds.frontLeftMetersPerSecond  / Constants.TICKMS_TO_MSEC : 0, DemandType.ArbitraryFeedForward, (Math.abs(wheelSpeeds.frontLeftMetersPerSecond ) < Constants.WHEEL_VELOCITY_DEADBAND ? 0 : Constants.FL_FF.ks) * (wheelSpeeds.frontLeftMetersPerSecond  > 0 ? 1 : -1));
        talFR.set(ControlMode.Velocity, Math.abs(wheelSpeeds.frontRightMetersPerSecond) > Constants.WHEEL_VELOCITY_DEADBAND ? wheelSpeeds.frontRightMetersPerSecond / Constants.TICKMS_TO_MSEC : 0, DemandType.ArbitraryFeedForward, (Math.abs(wheelSpeeds.frontRightMetersPerSecond) < Constants.WHEEL_VELOCITY_DEADBAND ? 0 : Constants.FR_FF.ks) * (wheelSpeeds.frontRightMetersPerSecond > 0 ? 1 : -1));
        talBL.set(ControlMode.Velocity, Math.abs(wheelSpeeds.rearLeftMetersPerSecond  ) > Constants.WHEEL_VELOCITY_DEADBAND ? wheelSpeeds.rearLeftMetersPerSecond   / Constants.TICKMS_TO_MSEC : 0, DemandType.ArbitraryFeedForward, (Math.abs(wheelSpeeds.rearLeftMetersPerSecond  ) < Constants.WHEEL_VELOCITY_DEADBAND ? 0 : Constants.BL_FF.ks) * (wheelSpeeds.rearLeftMetersPerSecond   > 0 ? 1 : -1));
        talBR.set(ControlMode.Velocity, Math.abs(wheelSpeeds.rearRightMetersPerSecond ) > Constants.WHEEL_VELOCITY_DEADBAND ? wheelSpeeds.rearRightMetersPerSecond  / Constants.TICKMS_TO_MSEC : 0, DemandType.ArbitraryFeedForward, (Math.abs(wheelSpeeds.rearRightMetersPerSecond ) < Constants.WHEEL_VELOCITY_DEADBAND ? 0 : Constants.BR_FF.ks) * (wheelSpeeds.rearRightMetersPerSecond  > 0 ? 1 : -1));
    }

    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
//        setWheelSpeeds(kinematics.toWheelSpeeds(targetSpeeds).normalize(Constants.MAX_WHEEL_VELOCITY));

//        var currentSpeeds = kinematics.toChassisSpeeds(getWheelSpeeds());
        var currentSpeeds = odometry.getChassisSpeeds();

        SmartDashboard.putNumber("vx", currentSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("vy", currentSpeeds.vyMetersPerSecond);

        setWheelSpeeds(kinematics.toWheelSpeeds(new ChassisSpeeds(
                    targetSpeeds.vxMetersPerSecond + Constants.VX_PID.calculate(currentSpeeds.vxMetersPerSecond, targetSpeeds.vxMetersPerSecond),
                    targetSpeeds.vyMetersPerSecond + Constants.VY_PID.calculate(currentSpeeds.vyMetersPerSecond, targetSpeeds.vyMetersPerSecond),
                    targetSpeeds.omegaRadiansPerSecond + Constants.OMEGA_PID.calculate(currentSpeeds.omegaRadiansPerSecond,  targetSpeeds.omegaRadiansPerSecond)))
                .normalize(Constants.MAX_WHEEL_VELOCITY));
    }

    public Pose2d getPose() {
//        return odometry.getPoseMeters();
//        return poseEstimator.getEstimatedPosition();
        return new Pose2d(odometry.getPoseMeters().getTranslation(), imu.getRotation2d().minus(gyroOffset));
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose);
        poseEstimator.resetPosition(pose, imu.getRotation2d());
        gyroOffset = imu.getRotation2d();
    }

    public XdriveKinematics getKinematics() {
        return kinematics;
    }
    

    /**
     * needs to be implemented
     * 
     * @return the current velocity of each wheel in m/s
     */
    public XdriveWheelSpeeds getWheelSpeeds() {
        return new XdriveWheelSpeeds(Constants.TICKMS_TO_MSEC * talFL.getSelectedSensorVelocity(),
                                     Constants.TICKMS_TO_MSEC * talFR.getSelectedSensorVelocity(),
                                     Constants.TICKMS_TO_MSEC * talBL.getSelectedSensorVelocity(),
                                     Constants.TICKMS_TO_MSEC * talBR.getSelectedSensorVelocity());
    }

    /**
     * Stops the motors when called
     */
    public void setStop(){
        talFL.set(ControlMode.PercentOutput, 0); // Stops the left motors
        talBL.set(ControlMode.PercentOutput, 0);

        talFR.set(ControlMode.PercentOutput, 0); // Stops the right motors
        talBR.set(ControlMode.PercentOutput, 0);
    }

    public Trajectory trajectoryFromJSON(String JSONPath) {
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(JSONPath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + JSONPath, ex.getStackTrace());
        }
        return trajectory;
    }

    public XdriveTrajectoryCommand makeTrajectoryCommand(Trajectory trajectory, BiFunction<Trajectory, Double, Rotation2d> desiredRotation) {
        return new XdriveTrajectoryCommand(
            trajectory,
            this::getPose,
            Constants.X_PID_CONTROLLER,
            Constants.Y_PID_CONTROLLER,
            Constants.THETA_PID_CONTROLLER,
            desiredRotation,
            this::setChassisSpeeds,
            this);
    }

    public XdriveTrajectoryCommand makeTrajectoryCommand(Trajectory trajectory) {
        return makeTrajectoryCommand(
            trajectory,
            (traj, time) -> Rotation2d.fromDegrees(0));
    }

    public XdriveTrajectoryCommand makeTrajectoryCommand(String path, BiFunction<Trajectory, Double, Rotation2d> desiredRotation) {
        return makeTrajectoryCommand(
            trajectoryFromJSON(path),
            desiredRotation);
    }

    public XdriveTrajectoryCommand makeTrajectoryCommand(String path) {
        return makeTrajectoryCommand(
            trajectoryFromJSON(path),
            (traj, time) -> Rotation2d.fromDegrees(0));
    }

    public Rotation2d getTeleopRotation() {
//        return getPose().getRotation();
        return imu.getRotation2d().minus(gyroOffset);
    }

    @Override
    public void periodic() {
//        odometry.update();
        odometry.updateWithGyro(imu.getRotation2d().minus(gyroOffset));
        poseEstimator.update(imu.getRotation2d(), getWheelSpeeds(), odometry.getPoseMeters());
        
        var estimatedPose = poseEstimator.getEstimatedPosition();

        SmartDashboard.putNumber("gyro_angle", imu.getRotation2d().minus(gyroOffset).getRadians());
        
        SmartDashboard.putNumber("imu_x", imu.getDisplacementX());
        SmartDashboard.putNumber("imu_y", imu.getDisplacementY());

        SmartDashboard.putNumber("imu_vx", imu.getVelocityX() * imu.getRotation2d().minus(gyroOffset).getCos());
        SmartDashboard.putNumber("imu_vy", imu.getVelocityY() * imu.getRotation2d().minus(gyroOffset).getSin());

        SmartDashboard.putNumber("pose_x", estimatedPose.getX());
        SmartDashboard.putNumber("pose_y", estimatedPose.getY());
        SmartDashboard.putNumber("pose_theta", estimatedPose.getRotation().getDegrees());

        var odometryPose = odometry.getPoseMeters();
        SmartDashboard.putNumber("odometry_x", odometryPose.getX());
        SmartDashboard.putNumber("odometry_y", odometryPose.getY());
        SmartDashboard.putNumber("odometry_theta", odometryPose.getRotation().getDegrees());

        field.setRobotPose(poseEstimator.getEstimatedPosition());
        field.getObject("odometry").setPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("FL velocity", Constants.TICKMS_TO_MSEC * talFL.getSelectedSensorVelocity());
        SmartDashboard.putNumber("FR velocity", Constants.TICKMS_TO_MSEC * talFR.getSelectedSensorVelocity());
        SmartDashboard.putNumber("BL velocity", Constants.TICKMS_TO_MSEC * talBL.getSelectedSensorVelocity());
        SmartDashboard.putNumber("BR velocity", Constants.TICKMS_TO_MSEC * talBR.getSelectedSensorVelocity());
    }
}