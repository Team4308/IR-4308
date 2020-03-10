package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.VelocityFlywheelCommand;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveTurn;
import frc.robot.subsystems.FlywheelSystem;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.TalonFXDriveSystem;

public class ToPowerPort1mr extends SequentialCommandGroup {

    public ToPowerPort1mr(TalonFXDriveSystem driveSystem, IntakeSystem intakeSystem, HopperSystem hopperSystem, FlywheelSystem flywheelSystem) {
        addCommands(
            new DriveDistance(-1.3, driveSystem),
            new DriveTurn(-45, driveSystem),
            new DriveDistance(-Math.sqrt(2.0), driveSystem),
            new DriveTurn(45, driveSystem),
            new DriveDistance(-0.5, driveSystem),
            new ParallelCommandGroup(new IntakeCommand(intakeSystem, () -> 0.5), new HopperCommand(hopperSystem, () -> 0.25), new VelocityFlywheelCommand(flywheelSystem, () -> -0.7))
        );
    }
    
}