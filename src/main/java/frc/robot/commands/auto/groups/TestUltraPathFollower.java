package frc.robot.commands.auto.groups;

import bbb.path.UltraPathFollower;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.TalonFXDriveSystem;

public class TestUltraPathFollower extends SequentialCommandGroup {

    public TestUltraPathFollower(TalonFXDriveSystem driveSystem) {
        addCommands(new UltraPathFollower("test", Constants.Config.Drive.MotionProfile.settings, driveSystem));
    }

}