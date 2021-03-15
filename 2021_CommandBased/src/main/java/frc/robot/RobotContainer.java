// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.TeleGroup;
import frc.robot.commands.XdriveTrajectoryCommand;

import frc.robot.OI;

import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Intake_s;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Define OI object
  public OI oi = new OI();
  
  // define subsystems
  private final Drive_s m_drive = new Drive_s();
  private final Intake_s m_intake = new Intake_s();
  
  private enum autoOptions {BARREL, BOUNCE, SLALOM, GALACTICSEARCH}

  //define a sendable chooser to select the autonomous command
  private SendableChooser<autoOptions> autoChooser = new SendableChooser<autoOptions>();

  private Timer timer = new Timer();
  private double startTime;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //add options to the chooser
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Auto Nav - Barrel path", autoOptions.BARREL);
    autoChooser.addOption("Auto Nav - Bounce path", autoOptions.BOUNCE);
    autoChooser.addOption("Auto Nav - Slalom path", autoOptions.SLALOM);
    autoChooser.addOption("Galactic Search", autoOptions.GALACTICSEARCH);

    //put the chooser on the dashboard
    SmartDashboard.putData(autoChooser);

    m_drive.setDefaultCommand(new TeleGroup(m_drive, oi));

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
    oi.getButton("REPLACE_ME").whileHeld(new InstantCommand(() -> m_intake.set(Constants.IN_SPEED), m_intake), true);
    oi.getButton("REPLACE_ME").whileHeld(new InstantCommand(() -> m_intake.set(Constants.OUT_SPEED), m_intake), true);

    //when pressed, initialize odometry and move to starting location for autonomous
    oi.getButton("REPLACE_ME").whenPressed(new SequentialCommandGroup(
                                              new InstantCommand(() -> m_drive.resetOdometry(Constants.INITIAL_POSE), m_drive),
                                              new SelectCommand(Map.ofEntries(
                                                                    Map.entry(autoOptions.BARREL, new XdriveTrajectoryCommand("paths/BarrelInit.wpilib.json", m_drive)),
                                                                    Map.entry(autoOptions.BOUNCE, new XdriveTrajectoryCommand("paths/BounceInit.wpilib.json", m_drive)),
                                                                    Map.entry(autoOptions.SLALOM, new XdriveTrajectoryCommand("paths/SlalomInit.wpilib.json", m_drive)),
                                                                    Map.entry(autoOptions.GALACTICSEARCH, new XdriveTrajectoryCommand("paths/GalacticSearchInit.wpilib.json", m_drive))),
                                                                autoChooser::getSelected)));
    //when pressed, run autonomous with a timer
    oi.getButton("REPLACE_ME").whenPressed(new SequentialCommandGroup(
                                              new InstantCommand(() -> startTime = timer.get()),
                                              new ParallelDeadlineGroup(
                                                  new SelectCommand(this::getAutonomousCommand),
                                                  new RunCommand(() -> SmartDashboard.putNumber("Autonomous Time", timer.get() - startTime)))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SelectCommand(Map.ofEntries(
                              Map.entry(autoOptions.BARREL, new XdriveTrajectoryCommand("paths/BarrelPath.wpilib.json", m_drive)),
                              Map.entry(autoOptions.BOUNCE, new XdriveTrajectoryCommand("paths/BouncePath.wpilib.json", m_drive)),
                              Map.entry(autoOptions.SLALOM, new XdriveTrajectoryCommand("paths/SlalomPath.wpilib.json", m_drive)),
                              Map.entry(autoOptions.GALACTICSEARCH, new ParallelDeadlineGroup(
                                                                      new SelectCommand(Map.ofEntries(
                                                                                          Map.entry("aRed", new XdriveTrajectoryCommand("paths/aRed.wpilib.json", m_drive)),
                                                                                          Map.entry("bRed", new XdriveTrajectoryCommand("paths/bRed.wpilib.json", m_drive)),
                                                                                          Map.entry("aBlue", new XdriveTrajectoryCommand("paths/aBlue.wpilib.json", m_drive)),
                                                                                          Map.entry("bBlue", new XdriveTrajectoryCommand("paths/bBlue.wpilib.json", m_drive))),
                                                                                        () -> NetworkTableInstance.getDefault().getTable("vision").getEntry("galacticSearchPath").getString("none")),
                                                                      new RunCommand(() -> m_intake.set(Constants.IN_SPEED), m_intake)))),
                            autoChooser::getSelected);
  }
}