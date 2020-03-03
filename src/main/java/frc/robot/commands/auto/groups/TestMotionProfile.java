package frc.robot.commands.auto.groups;

import bbb.path.PathFollowerSettings;
import bbb.path.UltraPathFollower;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.TalonFXDriveSystem;

public class TestMotionProfile extends SequentialCommandGroup {

    public TestMotionProfile(TalonFXDriveSystem driveSystem) {
        addCommands(new UltraPathFollower("test2", Constants.Config.Drive.MotionProfile.settings, driveSystem));
    }

}