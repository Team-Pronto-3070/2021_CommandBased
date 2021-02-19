package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Intake_s extends SubsystemBase{

    public Intake_s(){

        
    }

    /**
        Takes input to set speed of the intake motor
     */
    public void set(double input){

    }

    /**
        Actuates intake solenoid
     */
    public void setSolenoid(boolean input){
        Solenoid solenoid = new Solenoid(Constants.INTAKE_SOLENOID_PORT);
        solenoid.set(input);
    }
}