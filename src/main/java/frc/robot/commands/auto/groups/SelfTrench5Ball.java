package frc.robot.commands.auto.groups;

import bbb.path.UltraPathFollower;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.VelocityFlywheelCommand;
import frc.robot.subsystems.FlywheelSystem;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.RollerSystem;
import frc.robot.subsystems.TalonFXDriveSystem;

public class SelfTrench5Ball extends SequentialCommandGroup {

    public SelfTrench5Ball(TalonFXDriveSystem driveSystem, IntakeSystem intakeSystem, RollerSystem rollerSystem,
            HopperSystem hopperSystem, FlywheelSystem flywheelSystem) {
        addCommands(
                new ParallelRaceGroup(
                        new UltraPathFollower("5ballp1", Constants.Config.Drive.MotionProfile.settings, driveSystem),
                        new IntakeCommand(intakeSystem, () -> 0.5)),
                new UltraPathFollower("5ballp2", Constants.Config.Drive.MotionProfile.settings, driveSystem),
                new ParallelCommandGroup(new IntakeCommand(intakeSystem, () -> 0.5),
                        new RollerCommand(rollerSystem, () -> 0.5), new HopperCommand(hopperSystem, () -> 0.25),
                        new VelocityFlywheelCommand(flywheelSystem, () -> -0.7)));
    }

}