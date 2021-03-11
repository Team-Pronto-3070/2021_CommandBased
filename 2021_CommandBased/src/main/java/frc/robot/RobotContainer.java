// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.AutoNavBarrelGroup;
import frc.robot.commands.AutoNavBounceGroup;
import frc.robot.commands.AutoNavSlalomGroup;
import frc.robot.commands.GalacticSearchGroup;

import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Intake_s;

import frc.robot.OI;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // define subsystems
  private final Drive_s m_drive = new Drive_s();
  private final Intake_s m_intake = new Intake_s();

  // define commands
  private final Command m_autoNavBarrelGroup = new AutoNavBarrelGroup(m_drive);
  private final Command m_autoNavBounceGroup = new AutoNavBounceGroup(m_drive);
  private final Command m_autoNavSlalomGroup = new AutoNavSlalomGroup(m_drive);
  private final Command m_galacticSearchGroup = new GalacticSearchGroup(m_drive, m_intake);

  //define a sendable chooser to select the autonomous command
  SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  OI oi = new OI();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //add options to the chooser
    autoCommandChooser.setDefaultOption("None", null);
    autoCommandChooser.addOption("Auto Nav - Barrel path", m_autoNavBarrelGroup);
    autoCommandChooser.addOption("Auto Nav - Bounce path", m_autoNavBounceGroup);
    autoCommandChooser.addOption("Auto Nav - Slalom path", m_autoNavSlalomGroup);
    autoCommandChooser.addOption("Galactic Search", m_galacticSearchGroup);

    //put the chooser on the dashboard
    SmartDashboard.putData(autoCommandChooser);
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
    /**
     * runs the intake, INTAKE_SPEED is in constants
     */
    oi.getButton("btn1")
      .whileHeld(()-> Intake_s.set(INTAKE_SPEED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommandChooser.getSelected();
  }
}
