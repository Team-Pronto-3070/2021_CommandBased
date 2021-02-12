package frc.robot.subsystems;

//First imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//Motor control imports


//Local file imports
import frc.robot.Constants;

public class Drive_s extends SubsystemBase{

    TalonFX talFL; // Creates motor objects
    TalonFX talFR;
    TalonFX talBL;
    TalonFX talBR;
 
    /**
     * Constructor
     * 
    */ 
    
    public Drive_s(){
        talFL = new TalonFX(Constants.TAL_FL_PORT);
        talBL = new TalonFX(Constants.TAL_BL_PORT);
        talFR = new TalonFX(Constants.TAL_FR_PORT);
        talBR = new TalonFX(Constants.TAL_BR_PORT);
    }

    /**
     * Sets each motor speed separately. 
     * 
     * @param inputValues Double array for speeds of motors between -1 and 1 | [FL, FR, BL, BR]
     */
    public void setIndividual(double[] inputValues){
        talFL.set(ControlMode.PercentOutput, inputValues[0]);
    }

    /**
     * 
     * @param leftInput
     * @param rightInput
     */
    public void setTank(double leftInput, double rightInput){

    }

    public void setRotateDegree(double angle /*, Gyroscope reference*/){

    }

    public void setStop(){
        
    }
}