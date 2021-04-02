package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
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

    private Rotation2d teleopRotationOffset = new Rotation2d();

    private Field2d field = new Field2d();
    
    public Drive_s(){
        talFL = new TalonFX(Constants.TAL_FL_PORT);
        talBL = new TalonFX(Constants.TAL_BL_PORT);
        talFR = new TalonFX(Constants.TAL_FR_PORT);
        talBR = new TalonFX(Constants.TAL_BR_PORT);

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

        talFL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		talFR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		talBL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        talBR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        talFL.configSelectedFeedbackCoefficient(Constants.TICKMS_TO_MSEC);
        talFR.configSelectedFeedbackCoefficient(Constants.TICKMS_TO_MSEC);
        talBL.configSelectedFeedbackCoefficient(Constants.TICKMS_TO_MSEC);
        talBR.configSelectedFeedbackCoefficient(Constants.TICKMS_TO_MSEC);
        
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

    /**
     * uses PIDs to set the target velocity of each wheel
     *  
     * @param XdriveWheelSpeeds the target velocity of each wheel in meters/second
     */
//    public void setWheelSpeeds(XdriveWheelSpeeds speeds) {
//        XdriveWheelSpeeds currentSpeeds = getWheelSpeeds();
//        setIndividual(new XdriveWheelSpeeds(Constants.FL_PID.calculate(currentSpeeds.frontLeftMetersPerSecond, speeds.frontLeftMetersPerSecond) / Constants.MAX_WHEEL_VELOCITY,
//                                            Constants.FR_PID.calculate(currentSpeeds.frontRightMetersPerSecond, speeds.frontRightMetersPerSecond) / Constants.MAX_WHEEL_VELOCITY,
//                                            Constants.BL_PID.calculate(currentSpeeds.rearLeftMetersPerSecond, speeds.rearLeftMetersPerSecond) / Constants.MAX_WHEEL_VELOCITY,
//                                            Constants.BR_PID.calculate(currentSpeeds.rearRightMetersPerSecond, speeds.rearRightMetersPerSecond) / Constants.MAX_WHEEL_VELOCITY));
//    }

    /**
     * Sets the speed of each side of the robot, putting it into tank drive
     * 
     * @param leftInput Double for the speed of the left motors
     * @param rightInput Double for the speed of the right motors
     */
    public void setTank(double leftInput, double rightInput){

        talFL.set(ControlMode.PercentOutput, leftInput); // Set the left motors
        talBL.set(ControlMode.PercentOutput, leftInput);

        talFR.set(ControlMode.PercentOutput, rightInput); // Set the right motors
        talBR.set(ControlMode.PercentOutput, rightInput);

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
//        return poseEstimator.getEstimatedPosition();
//        return odometry.getPoseWith2Encoders(imu.getRotation2d());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose);
        poseEstimator.resetPosition(pose, imu.getRotation2d());
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
        return new XdriveWheelSpeeds(talFL.getSelectedSensorVelocity(),
                                     talFR.getSelectedSensorVelocity(),
                                     talBL.getSelectedSensorVelocity(),
                                     talBR.getSelectedSensorVelocity());
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

    public Rotation2d getTeleopRotation() {
        return getPose().getRotation().plus(teleopRotationOffset);
    }

    public void setTeleopRotationOffset(Rotation2d offset) {
        teleopRotationOffset = offset;
    }

    @Override
    public void periodic() {
        odometry.update();
//        poseEstimator.update(imu.getRotation2d(), getWheelSpeeds(), odometry.getPoseMeters());
        
        var estimatedPose = poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("gyro_angle", imu.getAngle());
        SmartDashboard.putNumber("pose_x", estimatedPose.getX());
        SmartDashboard.putNumber("pose_y", estimatedPose.getY());
        SmartDashboard.putNumber("pose_theta", estimatedPose.getRotation().getDegrees());

        var odometryPose = odometry.getPoseMeters();
        SmartDashboard.putNumber("odometry_x", odometryPose.getX());
        SmartDashboard.putNumber("odometry_y", odometryPose.getY());
        SmartDashboard.putNumber("odometry_theta", odometryPose.getRotation().getDegrees());

        field.setRobotPose(poseEstimator.getEstimatedPosition());
        field.getObject("odometry").setPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("FL velocity", talFL.getSelectedSensorVelocity());
        SmartDashboard.putNumber("FR velocity", talFR.getSelectedSensorVelocity());
        SmartDashboard.putNumber("BL velocity", talBL.getSelectedSensorVelocity());
        SmartDashboard.putNumber("BR velocity", talBR.getSelectedSensorVelocity());
    }
}