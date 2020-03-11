package frc.robot.commands.auto.groups;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TalonFXDriveSystem;

public class RamseteTest extends SequentialCommandGroup {
    public RamseteTest(TalonFXDriveSystem driveSystem) {
        addCommands(new RamseteCommand(
                TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()),
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d()),
                        new TrajectoryConfig(2, 4)),
                () -> driveSystem.odometry.getPoseMeters(), new RamseteController(), driveSystem.kinematics,
                driveSystem::setMotorVelocityMPS, driveSystem));
    }
}