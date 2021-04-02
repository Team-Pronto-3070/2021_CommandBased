package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.Constants;

public class Intake_s extends SubsystemBase{
    //Creates Talon
    TalonSRX talIN;
 

    public Intake_s(){
        talIN = new TalonSRX(Constants.TAL_INTAKE_PORT);
       // talIN.setInverted(true);

       talIN.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("test encoder", talIN.getSelectedSensorPosition());
    }

    /**
        Takes input to set speed of the intake motor
     
    @param velocity
    */
    public void set(double velocity){
        talIN.set(ControlMode.PercentOutput, velocity);
    }

}