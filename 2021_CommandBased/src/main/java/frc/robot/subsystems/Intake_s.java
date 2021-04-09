package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;

public class Intake_s extends SubsystemBase{
    TalonSRX talIN;

    public Intake_s() {
        talIN = new TalonSRX(Constants.TAL_INTAKE_PORT);
    }

    public void set(double velocity){
        talIN.set(ControlMode.PercentOutput, velocity);
    }
}