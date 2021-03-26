package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive_s;
import frc.robot.Constants;
import frc.robot.OI;

/** Relays to robot for movement according to joysticks
*
* Also needs DriveTrain Subsystem
*/

public class TeleopCommand extends CommandBase{
  //Drive & OI objects
  Drive_s _drive;
  OI _oi;

  public TeleopCommand(Drive_s _drive, OI _oi) {
    this._drive = _drive;
    this._oi = _oi;
    
    addRequirements(_drive);
  }

  //Overrides execute & periodically sends input to drivetrain
  @Override
  public void execute() {
    //Sets wheel speeds according to joystick values
    SmartDashboard.putNumber("x", _oi.getX());
    SmartDashboard.putNumber("y", _oi.getY());
    SmartDashboard.putNumber("theta", _oi.getTheta());
    _drive.setIndividual(_drive.getKinematics().toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                  (Math.abs(_oi.getX()) < Constants.JOY_STICK_VX_DEADZONE) ? 0 : _oi.getX() * Constants.VX_COEFFICENT,
                                  (Math.abs(_oi.getY()) < Constants.JOY_STICK_VY_DEADZONE) ? 0 : _oi.getY() * Constants.VY_COEFFICENT,
                                  (Math.abs(_oi.getTheta()) < Constants.JOY_STICK_OMEGA_DEADZONE) ? 0 : _oi.getTheta() * Constants.OMEGA_COEFFICENT,
                                  _drive.getPose().getRotation()))
                              .normalize(Constants.MAX_WHEEL_VELOCITY));
  }
}