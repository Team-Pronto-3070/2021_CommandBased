package frc.robot.subsystems;

//wpilib imports
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Motor control imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

//Local imports
import frc.robot.Constants;
import frc.robot.util.XdriveKinematics;
import frc.robot.util.XdriveOdometry;
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
    
    public Drive_s(){
        talFL = new TalonFX(Constants.TAL_FL_PORT);
        talBL = new TalonFX(Constants.TAL_BL_PORT);
        talFR = new TalonFX(Constants.TAL_FR_PORT);
        talBR = new TalonFX(Constants.TAL_BR_PORT);

        talFL.setInverted(InvertType.None);
        talFR.setInverted(InvertType.None);
        talBL.setInverted(InvertType.None);
        talBR.setInverted(InvertType.None);

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

        //initialize kinematics
        kinematics = new XdriveKinematics(new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(3 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(1 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(5 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(7 * Math.PI / 4)));
        
        //initialize odometry
        odometry = new XdriveOdometry();
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

    /**
     * uses PIDs to set the target velocity of each wheel
     *  
     * @param XdriveWheelSpeeds the target velocity of each wheel in meters/second
     */
    public void setWheelSpeeds(XdriveWheelSpeeds speeds) {
        XdriveWheelSpeeds currentSpeeds = getWheelSpeeds();
        setIndividual(new double[] {Constants.FL_PID.calculate(currentSpeeds.frontLeftMetersPerSecond, speeds.frontLeftMetersPerSecond) / Constants.MAX_WHEEL_VELOCITY,
                                    Constants.FR_PID.calculate(currentSpeeds.frontRightMetersPerSecond, speeds.frontRightMetersPerSecond) / Constants.MAX_WHEEL_VELOCITY,
                                    Constants.BL_PID.calculate(currentSpeeds.rearLeftMetersPerSecond, speeds.rearLeftMetersPerSecond) / Constants.MAX_WHEEL_VELOCITY,
                                    Constants.BR_PID.calculate(currentSpeeds.rearRightMetersPerSecond, speeds.rearRightMetersPerSecond) / Constants.MAX_WHEEL_VELOCITY});
    }

    //this should probably get moved to the teleop command because auto will call setWheelSpeeds directly
    public void driveAndTurn(double vx, double vy, double omega) {
        XdriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, odometry.getPoseMeters().getRotation()));
        wheelSpeeds.normalize(Constants.MAX_WHEEL_VELOCITY);
        setWheelSpeeds(wheelSpeeds);
    }

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

    @Override
    public void periodic() {
        odometry.update();
    }
}