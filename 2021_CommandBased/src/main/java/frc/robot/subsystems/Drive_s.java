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

    TalonFX talFL;
    TalonFX talFR;
    TalonFX talBL;
    TalonFX talBR;

    XdriveKinematics kinematics;
    XdriveOdometry odometry;
    private final XdrivePoseEstimator poseEstimator;
    private AHRS imu;
    private Field2d field = new Field2d();
    private Rotation2d gyroOffset = new Rotation2d();
    
    public Drive_s(){
        talFL = new TalonFX(Constants.Drive.TAL_FL_PORT);
        talBL = new TalonFX(Constants.Drive.TAL_BL_PORT);
        talFR = new TalonFX(Constants.Drive.TAL_FR_PORT);
        talBR = new TalonFX(Constants.Drive.TAL_BR_PORT);

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

        talFL.configOpenloopRamp(Constants.Drive.RAMP_TIME);
        talBL.configOpenloopRamp(Constants.Drive.RAMP_TIME);
        talFR.configOpenloopRamp(Constants.Drive.RAMP_TIME);
        talBR.configOpenloopRamp(Constants.Drive.RAMP_TIME);

        talFL.configClosedloopRamp(Constants.Drive.RAMP_TIME);
        talFR.configClosedloopRamp(Constants.Drive.RAMP_TIME);
        talBL.configClosedloopRamp(Constants.Drive.RAMP_TIME);
        talBR.configClosedloopRamp(Constants.Drive.RAMP_TIME);

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

        talFL.config_kP(0, Constants.Drive.FL_PID.getP() * Constants.Drive.TICKMS_TO_MSEC);
        talFL.config_kI(0, Constants.Drive.FL_PID.getI() * Constants.Drive.TICKMS_TO_MSEC);
        talFL.config_kD(0, Constants.Drive.FL_PID.getD() * Constants.Drive.TICKMS_TO_MSEC);
        talFL.config_kF(0, Constants.Drive.FL_FF.kv);
        
        talFR.config_kP(0, Constants.Drive.FR_PID.getP() * Constants.Drive.TICKMS_TO_MSEC);
        talFR.config_kI(0, Constants.Drive.FR_PID.getI() * Constants.Drive.TICKMS_TO_MSEC);
        talFR.config_kD(0, Constants.Drive.FR_PID.getD() * Constants.Drive.TICKMS_TO_MSEC);
        talFR.config_kF(0, Constants.Drive.FR_FF.kv);

        talBL.config_kP(0, Constants.Drive.BL_PID.getP() * Constants.Drive.TICKMS_TO_MSEC);
        talBL.config_kI(0, Constants.Drive.BL_PID.getI() * Constants.Drive.TICKMS_TO_MSEC);
        talBL.config_kD(0, Constants.Drive.BL_PID.getD() * Constants.Drive.TICKMS_TO_MSEC);
        talBL.config_kF(0, Constants.Drive.BL_FF.kv);

        talBR.config_kP(0, Constants.Drive.BR_PID.getP() * Constants.Drive.TICKMS_TO_MSEC);
        talBR.config_kI(0, Constants.Drive.BR_PID.getI() * Constants.Drive.TICKMS_TO_MSEC);
        talBR.config_kD(0, Constants.Drive.BR_PID.getD() * Constants.Drive.TICKMS_TO_MSEC);
        talBR.config_kF(0, Constants.Drive.BR_FF.kv);

        imu = new AHRS(Constants.Odometry.IMU_PORT);

        //initialize kinematics with relative locations of wheels
        kinematics = new XdriveKinematics(new Translation2d(Units.inchesToMeters(Constants.Drive.RADIUS_METERS), new Rotation2d(3 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.Drive.RADIUS_METERS), new Rotation2d(1 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.Drive.RADIUS_METERS), new Rotation2d(5 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.Drive.RADIUS_METERS), new Rotation2d(7 * Math.PI / 4)));
        
        odometry = new XdriveOdometry();

        poseEstimator = new XdrivePoseEstimator(imu.getRotation2d(),
                                                new Pose2d(),
                                                kinematics,
                                                Constants.Odometry.STATE_STD_DEVS,
                                                Constants.Odometry.IMU_STD_DEVS,
                                                Constants.Odometry.ODOMETRY_STD_DEVS);
        
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
        var FL_setpoint =  Math.abs(wheelSpeeds.frontLeftMetersPerSecond) > Constants.Drive.WHEEL_VELOCITY_DEADBAND ? wheelSpeeds.frontLeftMetersPerSecond : 0;
        var FR_setpoint =  Math.abs(wheelSpeeds.frontRightMetersPerSecond) > Constants.Drive.WHEEL_VELOCITY_DEADBAND ? wheelSpeeds.frontRightMetersPerSecond : 0;
        var BL_setpoint =  Math.abs(wheelSpeeds.rearLeftMetersPerSecond) > Constants.Drive.WHEEL_VELOCITY_DEADBAND ? wheelSpeeds.rearLeftMetersPerSecond : 0;
        var BR_setpoint =  Math.abs(wheelSpeeds.rearRightMetersPerSecond) > Constants.Drive.WHEEL_VELOCITY_DEADBAND ? wheelSpeeds.rearRightMetersPerSecond : 0;
        
        SmartDashboard.putNumber("FL_SETPOINT", FL_setpoint);
        SmartDashboard.putNumber("FR_SETPOINT", FR_setpoint);
        SmartDashboard.putNumber("BL_SETPOINT", BL_setpoint);
        SmartDashboard.putNumber("BR_SETPOINT", BR_setpoint);

        talFL.set(ControlMode.Velocity, FL_setpoint / Constants.Drive.TICKMS_TO_MSEC, DemandType.ArbitraryFeedForward, Constants.Drive.FL_FF.ks * Math.signum(FL_setpoint));
        talFR.set(ControlMode.Velocity, FR_setpoint / Constants.Drive.TICKMS_TO_MSEC, DemandType.ArbitraryFeedForward, Constants.Drive.FR_FF.ks * Math.signum(FR_setpoint));
        talBL.set(ControlMode.Velocity, BL_setpoint / Constants.Drive.TICKMS_TO_MSEC, DemandType.ArbitraryFeedForward, Constants.Drive.BL_FF.ks * Math.signum(BL_setpoint));
        talBR.set(ControlMode.Velocity, BR_setpoint / Constants.Drive.TICKMS_TO_MSEC, DemandType.ArbitraryFeedForward, Constants.Drive.BR_FF.ks * Math.signum(BR_setpoint));
    }

    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        var currentSpeeds = odometry.getChassisSpeeds();

        SmartDashboard.putNumber("vx", currentSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("vy", currentSpeeds.vyMetersPerSecond);

        setWheelSpeeds(kinematics.toWheelSpeeds(new ChassisSpeeds(
                    targetSpeeds.vxMetersPerSecond + Constants.Drive.VX_PID.calculate(currentSpeeds.vxMetersPerSecond, targetSpeeds.vxMetersPerSecond),
                    targetSpeeds.vyMetersPerSecond + Constants.Drive.VY_PID.calculate(currentSpeeds.vyMetersPerSecond, targetSpeeds.vyMetersPerSecond),
                    targetSpeeds.omegaRadiansPerSecond + Constants.Drive.OMEGA_PID.calculate(currentSpeeds.omegaRadiansPerSecond,  targetSpeeds.omegaRadiansPerSecond)))
                .normalize(Constants.Drive.MAX_WHEEL_VELOCITY));
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
    
    public XdriveWheelSpeeds getWheelSpeeds() {
        return new XdriveWheelSpeeds(Constants.Drive.TICKMS_TO_MSEC * talFL.getSelectedSensorVelocity(),
                                     Constants.Drive.TICKMS_TO_MSEC * talFR.getSelectedSensorVelocity(),
                                     Constants.Drive.TICKMS_TO_MSEC * talBL.getSelectedSensorVelocity(),
                                     Constants.Drive.TICKMS_TO_MSEC * talBR.getSelectedSensorVelocity());
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
            Constants.Drive.X_PID_CONTROLLER,
            Constants.Drive.Y_PID_CONTROLLER,
            Constants.Drive.THETA_PID_CONTROLLER,
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

        SmartDashboard.putNumber("FL velocity", Constants.Drive.TICKMS_TO_MSEC * talFL.getSelectedSensorVelocity());
        SmartDashboard.putNumber("FR velocity", Constants.Drive.TICKMS_TO_MSEC * talFR.getSelectedSensorVelocity());
        SmartDashboard.putNumber("BL velocity", Constants.Drive.TICKMS_TO_MSEC * talBL.getSelectedSensorVelocity());
        SmartDashboard.putNumber("BR velocity", Constants.Drive.TICKMS_TO_MSEC * talBR.getSelectedSensorVelocity());
    }
}