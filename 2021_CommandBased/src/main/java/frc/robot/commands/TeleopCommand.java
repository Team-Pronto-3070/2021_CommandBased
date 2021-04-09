package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive_s;
import frc.robot.util.XdriveWheelSpeeds;
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

  Timer timer = new Timer();
  double prevTime = timer.get();
  XdriveWheelSpeeds m_prevSpeeds = new XdriveWheelSpeeds();

  public TeleopCommand(Drive_s _drive, OI _oi) {
    this._drive = _drive;
    this._oi = _oi;
    
    addRequirements(_drive);
  }

  @Override
  public void execute() {
    //Sets wheel speeds according to joystick values
    SmartDashboard.putNumber("teleop_x", _oi.getX());
    SmartDashboard.putNumber("teleop_y", _oi.getY());
    SmartDashboard.putNumber("teleop_theta", _oi.getTheta());

    _drive.setChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                (Math.abs(_oi.getX()) < Constants.JOY_STICK_VX_DEADZONE) ? 0 : _oi.getX() * Constants.VX_COEFFICENT,
                (Math.abs(_oi.getY()) < Constants.JOY_STICK_VY_DEADZONE) ? 0 : _oi.getY() * Constants.VY_COEFFICENT,
                (Math.abs(_oi.getTheta()) < Constants.JOY_STICK_OMEGA_DEADZONE) ? 0 : _oi.getTheta() * Constants.OMEGA_COEFFICENT,
                _drive.getTeleopRotation()));
  }
}