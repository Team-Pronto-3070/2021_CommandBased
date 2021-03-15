package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_s;
import frc.robot.Constants;
import frc.robot.OI;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import frc.robot.util.XdriveWheelSpeeds;
/** Relays to robot for movement according to joysticks
*
* Also needs DriveTrain Subsystem
*/

public class TeleopCommand extends CommandBase{
  //Drive & OI objects
  Drive_s _drive;
  OI _oi;

  //Raw joystick values
  double vx;
  double vy;
  double omega;

  public TeleopCommand(Drive_s _drive, OI _oi) {
    this._drive = _drive;
    this._oi = _oi;
    
    addRequirements(_drive);
  }

  //Overrides execute & periodically sends input to drivetrain
  @Override
  public void execute() {
    //Sets deadzones for the joystick values
    vx = (Math.abs(_oi.getX()) < Constants.JOY_STICK_VX_DEADZONE) ? 0 : _oi.getX();
    vy = (Math.abs(_oi.getY()) < Constants.JOY_STICK_VY_DEADZONE) ? 0 : _oi.getY();
    omega = (Math.abs(_oi.getTheta()) < Constants.JOY_STICK_OMEGA_DEADZONE) ? 0 : _oi.getTheta();

    //Sets wheel speeds according to joystick values
    XdriveWheelSpeeds wheelSpeeds = _drive.getKinematics().toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, _drive.getPose().getRotation()));
    wheelSpeeds.normalize(Constants.MAX_WHEEL_VELOCITY);
    _drive.setWheelSpeeds(wheelSpeeds);
  }
}
 







