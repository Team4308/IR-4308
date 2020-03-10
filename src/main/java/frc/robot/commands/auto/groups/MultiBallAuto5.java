package frc.robot.commands.auto.groups;

import bbb.path.UltraPathFollower;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.FlywheelSystem;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.TalonFXDriveSystem;

public class MultiBallAuto5 extends SequentialCommandGroup {

    public MultiBallAuto5(TalonFXDriveSystem driveSystem, IntakeSystem intakeSystem, HopperSystem hopperSystem, FlywheelSystem flywheelSystem) {
        addCommands(new UltraPathFollower("5ballauto", Constants.Config.Drive.MotionProfile.settings, driveSystem));
    }

}