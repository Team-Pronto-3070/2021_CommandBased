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

  //Overrides execute & periodically sends input to drivetrain
  @Override
  public void execute() {
    //Sets wheel speeds according to joystick values
    SmartDashboard.putNumber("teleop_x", _oi.getX());
    SmartDashboard.putNumber("teleop_y", _oi.getY());
    SmartDashboard.putNumber("teleop_theta", _oi.getTheta());

//    _drive.setIndividual(
//    _drive.setWheelSpeeds(
//    closedLoop(
    _drive.setChassisSpeeds(
//                    _drive.getKinematics().toWheelSpeeds(
                              ChassisSpeeds.fromFieldRelativeSpeeds(
                                  (Math.abs(_oi.getX()) < Constants.JOY_STICK_VX_DEADZONE) ? 0 : _oi.getX() * Constants.VX_COEFFICENT,
                                  (Math.abs(_oi.getY()) < Constants.JOY_STICK_VY_DEADZONE) ? 0 : _oi.getY() * Constants.VY_COEFFICENT,
                                  (Math.abs(_oi.getTheta()) < Constants.JOY_STICK_OMEGA_DEADZONE) ? 0 : _oi.getTheta() * Constants.OMEGA_COEFFICENT,
                                  _drive.getTeleopRotation()))
//                              .normalize(Constants.MAX_WHEEL_VELOCITY))
                                                                ;
  }

  private void closedLoop(XdriveWheelSpeeds targetWheelSpeeds) {
    var frontLeftSpeedSetpoint = 4 * targetWheelSpeeds.frontLeftMetersPerSecond;
    var rearLeftSpeedSetpoint = 4 * targetWheelSpeeds.rearLeftMetersPerSecond;
    var frontRightSpeedSetpoint = 4 * targetWheelSpeeds.frontRightMetersPerSecond;
    var rearRightSpeedSetpoint = 4 * targetWheelSpeeds.rearRightMetersPerSecond;

    SmartDashboard.putNumber("FL_SETPOINT", frontLeftSpeedSetpoint);
    SmartDashboard.putNumber("FR_SETPOINT", frontRightSpeedSetpoint);
    SmartDashboard.putNumber("BL_SETPOINT", rearLeftSpeedSetpoint);
    SmartDashboard.putNumber("BR_SETPOINT", rearRightSpeedSetpoint);

    double frontLeftOutput;
    double rearLeftOutput;
    double frontRightOutput;
    double rearRightOutput;

    var m_frontLeftController = Constants.FL_PID;
    var m_frontRightController = Constants.FR_PID;
    var m_rearLeftController = Constants.BL_PID;
    var m_rearRightController = Constants.BR_PID;

    var m_currentWheelSpeeds = _drive.getWheelSpeeds();

    double curTime = timer.get();
    double dt = curTime - prevTime;
    prevTime = curTime;

      final double frontLeftFeedforward =
          Constants.FL_FF.calculate(
              frontLeftSpeedSetpoint);

      final double rearLeftFeedforward =
          Constants.BL_FF.calculate(
              rearLeftSpeedSetpoint);

      final double frontRightFeedforward =
          Constants.FR_FF.calculate(
              frontRightSpeedSetpoint);

      final double rearRightFeedforward =
          Constants.BR_FF.calculate(
              rearRightSpeedSetpoint);

      frontLeftOutput =
          frontLeftFeedforward
              + m_frontLeftController.calculate(
                  m_currentWheelSpeeds.frontLeftMetersPerSecond, frontLeftSpeedSetpoint);

      rearLeftOutput =
          rearLeftFeedforward
              + m_rearLeftController.calculate(
                  m_currentWheelSpeeds.rearLeftMetersPerSecond, rearLeftSpeedSetpoint);

      frontRightOutput =
          frontRightFeedforward
              + m_frontRightController.calculate(
                  m_currentWheelSpeeds.frontRightMetersPerSecond, frontRightSpeedSetpoint);

      rearRightOutput =
          rearRightFeedforward
              + m_rearRightController.calculate(
                  m_currentWheelSpeeds.rearRightMetersPerSecond, rearRightSpeedSetpoint);

      _drive.setIndividual(
          new XdriveWheelSpeeds(
              frontLeftOutput, frontRightOutput, rearLeftOutput, rearRightOutput));
  }
}