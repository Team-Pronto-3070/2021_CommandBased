package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import frc.robot.OI;
import edu.wpi.first.wpilibj.Joystick;

public class TeleGroup extends ParallelCommandGroup{

    OI oi;
    Joystick controller = oi.getController();

    public TeleGroup(Drive_s drive, OI oi){
       this.oi = oi;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        //Perform some calculation here to determine what to input to drive.driveAndTurn(vx,vy,omega);
    }

}
