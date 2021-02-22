package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Units;
//First imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//Motor control imports


//Local file imports
import frc.robot.Constants;
import frc.robot.util.XdriveKinematics;
import frc.robot.util.XdriveWheelSpeeds;

public class Drive_s extends SubsystemBase{

    TalonFX talFL; // Creates motor objects
    TalonFX talFR;
    TalonFX talBL;
    TalonFX talBR;

    //create kinematics
    XdriveKinematics kinematics;

    /**
     * Constructor
    */ 
    
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

        //initialize kinematics
        kinematics = new XdriveKinematics(new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(3 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(1 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(5 * Math.PI / 4)),
                                          new Translation2d(Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES), new Rotation2d(7 * Math.PI / 4)));
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

    public void driveAndTurn(double vx, double vy, double omega) {
        XdriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(vx, vy, omega));
        wheelSpeeds.normalize(1.0);
        setIndividual(new double[] {wheelSpeeds.frontLeftMetersPerSecond,
                                    wheelSpeeds.frontRightMetersPerSecond,
                                    wheelSpeeds.rearLeftMetersPerSecond,
                                    wheelSpeeds.rearRightMetersPerSecond});
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

    public void setRotateDegree(double angle /*, Gyroscope reference*/){

    }

    public void setStop(){
        
    }
}