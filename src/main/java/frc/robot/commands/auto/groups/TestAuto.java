package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveTurn;
import frc.robot.subsystems.TalonFXDriveSystem;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(TalonFXDriveSystem driveSystem) {
        addCommands(
            new DriveDistance(2, driveSystem),
            new DriveTurn(180, driveSystem),
            new DriveDistance(1, driveSystem),
            new DriveTurn(180, driveSystem),
            new DriveDistance(-1, driveSystem)
        );
    }
    
}