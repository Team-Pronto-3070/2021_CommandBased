// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.TeleopCommand;
import frc.robot.commands.XdriveTrajectoryCommand;

import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Intake_s;
import frc.robot.util.XdriveKinematicsConstraint;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Define OI object
  public OI oi = new OI();

  public Vision vis = new Vision();
  
  // define subsystems
  private final Drive_s m_drive = new Drive_s();
  private final Intake_s m_intake = new Intake_s();
  
  private enum autoOptions {BARREL, BOUNCE, SLALOM, GALACTICSEARCH, TEST1, TEST2}

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
    autoChooser.addOption("test 1", autoOptions.TEST1);
    autoChooser.addOption("test 2", autoOptions.TEST2);

    //put the chooser on the dashboard
    SmartDashboard.putData(autoChooser);

    m_drive.setDefaultCommand(new TeleopCommand(m_drive, oi));
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.set(0), m_intake));

    // Configure the button bindings
    configureButtonBindings();
  }

  private Trajectory generateInitTrajectory(String path) {
    return TrajectoryGenerator.generateTrajectory(Constants.INITIAL_POSE,
                                                  List.of(),
                                                  m_drive.trajectoryFromJSON(path).getInitialPose(),
                                                  new TrajectoryConfig(1 /*max velocity*/, 0.5 /*max acceleration*/)
                                                    .addConstraint(new XdriveKinematicsConstraint(m_drive.getKinematics(), 1 /*max velocity*/)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    oi.addButton("intake_in", 1);
    oi.addButton("intake_out", 2);
    oi.addButton("autoInit", 11);
    oi.addButton("auto", 12);
    oi.addButton("teleopRotationOffset", 7);
    oi.addButton("resetPose", 8);

    oi.addButton("setARed", 3);
    oi.addButton("setABlue", 4);
    oi.addButton("setBRed", 5);
    oi.addButton("setBBlue", 6);
    oi.addButton("selectPath", 10);

    oi.getButton("setARed").whenPressed(new InstantCommand(() -> vis.updateProfile("aRed")));
    oi.getButton("setABlue").whenPressed(new InstantCommand(() -> vis.updateProfile("aBlue")));
    oi.getButton("setBRed").whenPressed(new InstantCommand(() -> vis.updateProfile("bRed")));
    oi.getButton("setBBlue").whenPressed(new InstantCommand(() -> vis.updateProfile("bBlue")));
    oi.getButton("selectPath").whenPressed(new InstantCommand(() -> System.out.println("\n\n"+vis.selectPath()+"\n\n")));

    // Referencing the added buttons when pressed
    oi.getButton("intake_in").whileHeld(new InstantCommand(() -> m_intake.set(Constants.IN_SPEED), m_intake), true);
    oi.getButton("intake_out").whileHeld(new InstantCommand(() -> m_intake.set(Constants.OUT_SPEED), m_intake), true);

    //when pressed, initialize odometry and move to starting location for autonomous
    oi.getButton("autoInit").whenPressed(new SequentialCommandGroup(
                                              new InstantCommand(() -> m_drive.resetOdometry(Constants.INITIAL_POSE), m_drive),
                                              new SelectCommand(Map.ofEntries(
                                                                    Map.entry(autoOptions.BARREL, new XdriveTrajectoryCommand(generateInitTrajectory("paths/BarrelPath.wpilib.json"), m_drive)),
                                                                    Map.entry(autoOptions.BOUNCE, new XdriveTrajectoryCommand(generateInitTrajectory("paths/BouncePath.wpilib.json"), m_drive)),
                                                                    Map.entry(autoOptions.SLALOM, new XdriveTrajectoryCommand(generateInitTrajectory("paths/SlalomPath.wpilib.json"), m_drive)),
                                                                    Map.entry(autoOptions.GALACTICSEARCH, new XdriveTrajectoryCommand(generateInitTrajectory("paths/aRed.wpilib.json"), m_drive))),
                                                                autoChooser::getSelected)));
    //when pressed, run autonomous with a timer
    oi.getButton("auto").whenPressed(new SequentialCommandGroup(
                                              new InstantCommand(() -> startTime = timer.get()),
                                              new ParallelDeadlineGroup(
                                                  getAutonomousCommand(),
                                                  new RunCommand(() -> SmartDashboard.putNumber("Autonomous Time", timer.get() - startTime)))));
    oi.getButton("teleopRotationOffset").whenPressed(new InstantCommand(() -> m_drive.setTeleopRotationOffset(m_drive.getPose().getRotation()), m_drive));
    oi.getButton("resetPose").whenPressed(new InstantCommand(() -> m_drive.resetOdometry(new Pose2d()), m_drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    vis.readPaths();
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
                                                                                          vis::selectPath),
                                                                      new RunCommand(() -> m_intake.set(Constants.IN_SPEED), m_intake))),
                              
                              Map.entry(autoOptions.TEST1, new XdriveTrajectoryCommand(
                                    TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                           List.of(),
                                                                           new Pose2d(1, 0, new Rotation2d(0)),
                                              new TrajectoryConfig(1 /*max velocity*/, 0.5 /*max acceleration*/).addConstraint(new XdriveKinematicsConstraint(m_drive.getKinematics(), 1 /*max velocity*/))),
                                  m_drive)),
                              Map.entry(autoOptions.TEST2, new XdriveTrajectoryCommand(
                                    TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                           List.of(),
                                                                           new Pose2d(-1, 0, new Rotation2d(0)),
                                              new TrajectoryConfig(1 /*max velocity*/, 0.5 /*max acceleration*/).addConstraint(new XdriveKinematicsConstraint(m_drive.getKinematics(), 1 /*max velocity*/))),
                                  m_drive))),
                            autoChooser::getSelected);
  }
}