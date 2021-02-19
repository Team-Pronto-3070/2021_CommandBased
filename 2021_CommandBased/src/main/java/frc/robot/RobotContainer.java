// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoGroup;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake_s;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive_s drive = new Drive_s();
  private final Intake_s intake = new Intake_s();

  private final AutoGroup autoCommand = new AutoGroup(drive,intake);

  OI oi = new OI();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    oi.addButton("btn1", 1);
    oi.addButton("btn3", 3);
    oi.addButton("btn2", 2);
    oi.addButton("btn4", 4);
    oi.addButton("btn5", 5);


    // Referencing the added buttons when pressed
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;

    return autoCommand;
  }
}
