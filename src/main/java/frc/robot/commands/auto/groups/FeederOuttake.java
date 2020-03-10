package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.VelocityFlywheelCommand;
import frc.robot.subsystems.FlywheelSystem;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.IntakeSystem;

public class FeederOuttake extends SequentialCommandGroup {
    public FeederOuttake(IntakeSystem intakeSystem, HopperSystem hopperSystem, FlywheelSystem flywheelSystem) {
        addCommands(
            new ParallelCommandGroup(new IntakeCommand(intakeSystem, () -> 0.5), new HopperCommand(hopperSystem, () -> 0.25), new VelocityFlywheelCommand(flywheelSystem, () -> -0.7))
        );
    }
}