package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Drive_s;

public class AutoNavBounceGroup extends SequentialCommandGroup {
    
    public AutoNavBounceGroup(Drive_s drive) {
                String trajectoryJSON = "paths/BouncePath.wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        addRequirements(drive);
        addCommands(new XdriveTrajectoryCommand(trajectory, drive));

    }
}