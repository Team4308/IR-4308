/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import bbb.control.JoystickHelper;
import bbb.control.XBoxWrapper;
import bbb.math.bbbVector2;
import bbb.utils.bbbDoubleUtils;
import frc.robot.subsystems.TalonSRXDriveSystem;
import frc.robot.subsystems.TalonSRXDriveSystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    /**
     * Subsystems
     */
    // NavX on MXP (Must Be Declared Before Everything)
    public final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    // Drivetrain
    public final TalonSRXDriveSystem m_driveSystem = new TalonSRXDriveSystem(this.ahrs);

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
    public bbbVector2 getDriveControl() {
        double throttle = bbbDoubleUtils.normalize(driveStick.getLeftY());
        double turn = bbbDoubleUtils.normalize(driveStick.getRightX());

        bbbVector2 control = new bbbVector2(turn, throttle);
        control = JoystickHelper.ScaledAxialDeadzone(control);
        control = JoystickHelper.alternateScaleStick(control, 2);
        control = JoystickHelper.clampStick(control);

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
