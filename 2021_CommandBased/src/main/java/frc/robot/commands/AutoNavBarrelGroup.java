package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Drive_s;

public class AutoNavBarrelGroup extends SequentialCommandGroup {
    
    public AutoNavBarrelGroup(Drive_s drive) {
        String JSONPath = "paths/BarrelPath.wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(JSONPath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + JSONPath, ex.getStackTrace());
        }

        addRequirements(drive);
        addCommands(new XdriveTrajectoryCommand(trajectory, drive));
    }
}