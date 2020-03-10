package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSystem;

public class FeederIntake extends SequentialCommandGroup {
    public FeederIntake(IntakeSystem intakeSystem) {
        addCommands(
            new ParallelCommandGroup(new IntakeCommand(intakeSystem, () -> -0.5))
        );
    }
}