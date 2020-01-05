/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import bbb.control.JoystickHelper;
import bbb.control.XBoxWrapper;
import bbb.math.bbbVector2;
import bbb.utils.bbbDoubleUtils;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    /**
     * Subsystems
     */
    // Drivetrain
    public final DriveSystem m_driveSystem = new DriveSystem();

    /**
     * Commands
     */
    // Normal Arcade Drive
    public final RunCommand normalArcadeDriveCommand = new RunCommand(
            () -> m_driveSystem.NormalArcadeDrive(getDriveControl()), m_driveSystem);

    /**
     * Human Controllers
     */
    public XBoxWrapper driveStick = new XBoxWrapper(0);
    public XBoxWrapper controlStick = new XBoxWrapper(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    // Configure Button Bindings
    private void configureButtonBindings() {
    }

    /**
     * Filtered Outputs
     */
    // Drive Control
    private bbbVector2 getDriveControl() {
        double throttle = bbbDoubleUtils.normalize(-driveStick.getLeftY());
        double turn = bbbDoubleUtils.normalize(driveStick.getRightX());

        bbbVector2 control = new bbbVector2(turn, throttle);
        control = JoystickHelper.DeadbandRadial(control);
        control = JoystickHelper.scaleStick(control, 2);

        return control;
    }

    /**
     * Misc
     */
    // Return Auto Command
    public Command getAutonomousCommand() {
        return null;
    }
}
