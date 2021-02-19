package frc.robot;

/* Imports */
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants;

import java.util.HashMap;

/**
 * Framework for the operator interface for the robot.
 * 
 * Contains necessary sensors and manual control objects (i.e. joysticks).
 */
public class OI
{
    /* Class Variable Declaration */
    HashMap<String, Joystick> joyMap;
    HashMap<String, JoystickButton> buttons;
    
    Joystick joystick;

    /**
     * Constructs the Operator Interface.
     */
    public OI()
    {
        /* Class Variable Instantiation */
        

        joyMap = new HashMap<>();
        buttons = new HashMap<>();

    
        joystick = new Joystick(Constants.JOY_PORT);

    }

    /**
     * @param name of a joystick in the OI.
     * @return A desired Joystick object from the OI.
     */
    public Joystick getController()
    {
       return joystick;
    }

    /**
     * Add a particular button to the operator interface.
     * Although these buttons could technically be accessed using the joystick object(s),
     * this implementation makes it easier for the developers to understand
     * the functionality of each button and allows for smoother implementation
     * of configureButtonBindings() in RobotContainer.
     * 
     * @param name of the button.
     * @param joystick that the button belongs to.
     * @param number assigned to the button by the controller in the Driver Station.
     */
    public void addButton(String name, int number)
    {
        buttons.put(name, new JoystickButton(joystick, number));
    }

    /**
     * @param name of a joystick button in the OI.
     * @return A desired JoystickButton object from the OI.
     */
    public JoystickButton getButton(String name)
    {
        return buttons.get(name);
    }
}