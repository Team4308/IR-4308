package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.motionprofile.MotionProfileCommand;
import frc.robot.subsystems.TalonFXDriveSystem;

public class TestMotionProfile extends SequentialCommandGroup {

    public TestMotionProfile(TalonFXDriveSystem driveSystem) {
        addCommands(
            new MotionProfileCommand("test", driveSystem)
        );
    }
    
}