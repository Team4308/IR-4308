package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.VelocityFlywheelCommand;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.FlywheelSystem;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.RollerSystem;
import frc.robot.subsystems.TalonFXDriveSystem;

public class Steal extends SequentialCommandGroup {

    public Steal(TalonFXDriveSystem driveSystem, IntakeSystem intakeSystem, RollerSystem rollerSystem) {
        addCommands(
            new ParallelRaceGroup(new DriveDistance(3.4, driveSystem), new IntakeCommand(intakeSystem, () -> 0.5), new RollerCommand(rollerSystem, () -> 0.5)),
            new ParallelRaceGroup(new DriveDistance(-1, driveSystem), new IntakeCommand(intakeSystem, () -> 0.5), new RollerCommand(rollerSystem, () -> 0.5))
        );
    }
    
}