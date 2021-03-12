package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Intake_s;
import frc.robot.Constants;

public class GalacticSearchGroup extends SequentialCommandGroup {
    
    public GalacticSearchGroup(Drive_s drive, Intake_s intake) {
        String JSONPath;

        switch (NetworkTableInstance.getDefault().getTable("vision").getEntry("galacticSearchPath").getString("none")) {
            case "aRed":
                JSONPath = "paths/aRed.wpilib.json";
                break;
            case "aBlue":
                JSONPath = "paths/aBlue.wpilib.json";
                break;
            case "bRed":
                JSONPath = "paths/bRed.wpilib.json";
                break;
            case "bBlue":
                JSONPath = "paths/bBlue.wpilib.json";
                break;
            default:
                JSONPath = "paths/emptyPath.wpilib.json";
                break;
        }

        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(JSONPath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + JSONPath, ex.getStackTrace());
        }

        addCommands(new ParallelDeadlineGroup(
                        new XdriveTrajectoryCommand(trajectory, drive),
                        new RunCommand(() -> intake.set(Constants.INTAKE_SPEED), intake)));
    }
}