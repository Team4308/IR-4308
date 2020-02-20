/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import bbb.control.JoystickHelper;
import bbb.control.XBoxWrapper;
import bbb.math.bbbVector2;
import bbb.utils.bbbDoubleUtils;
import bbb.wrapper.LogSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NormalArcadeDriveCommand;
import frc.robot.commands.RotatePanelCommand;
import frc.robot.commands.SelectColorCommand;
import frc.robot.commands.VelocityArcadeDriveCommand;
import frc.robot.subsystems.sensors.ColorSensor;
import frc.robot.subsystems.ControlPanelSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.TalonFXDriveSystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

    // NavX on MXP (Must Be Declared Before Everything)
    public final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    /**
     * Subsystems
     */
    public ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

    // Drivetrain
    private final TalonFXDriveSystem m_driveSystem;

    // Color Sensor
    private final ColorSensor m_colorSensor;

    // Control Panel
    private final ControlPanelSystem m_controlPanelSystem;

    // Intake
    private final IntakeSystem m_intakeSystem;

    /**
     * Commands
     */
    // Normal Arcade Drive
    private final NormalArcadeDriveCommand normalArcadeDriveCommand;
    // Velocity Arcade Drive
    private final VelocityArcadeDriveCommand velocityArcadeDriveCommand;
    // Intake
    private final IntakeCommand intakeCommand;

    /**
     * Human Controllers
     */
    public XBoxWrapper driveStick = new XBoxWrapper(0);
    public XBoxWrapper controlStick = new XBoxWrapper(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        /**
         * Init Subsystems
         */
        m_driveSystem = new TalonFXDriveSystem(this.ahrs);
        subsystems.add(m_driveSystem);

        m_colorSensor = new ColorSensor();
        subsystems.add(m_colorSensor);

        m_controlPanelSystem = new ControlPanelSystem();
        subsystems.add(m_controlPanelSystem);

        m_intakeSystem = new IntakeSystem();
        subsystems.add(m_intakeSystem);

        /**
         * Init Commands
         */
        // Normal Arcade Drive
        normalArcadeDriveCommand = new NormalArcadeDriveCommand(m_driveSystem, () -> getDriveControl());
        // Velocity Arcade Drive
        velocityArcadeDriveCommand = new VelocityArcadeDriveCommand(m_driveSystem, () -> getDriveControl());
        // Intake 
        intakeCommand = new IntakeCommand(m_intakeSystem, () -> getIntakeControl());

        m_driveSystem.setDefaultCommand(velocityArcadeDriveCommand);


        // Configure the button bindings
        configureButtonBindings();
    }

    // Configure Button Bindings
    private void configureButtonBindings() {
        driveStick.Back.whenPressed(new InstantCommand(() -> m_driveSystem.resetSensors(), m_driveSystem));
        controlStick.Back.whenPressed(new RotatePanelCommand());
        controlStick.Start.whenPressed(new SelectColorCommand());
        controlStick.RB.whenPressed(new InstantCommand(() -> m_intakeSystem.isFlipped = 1, m_intakeSystem));
        controlStick.LB.whenPressed(new InstantCommand(() -> m_intakeSystem.isFlipped = -1, m_intakeSystem));
    }

    /**
     * Filtered Outputs
     */
    // Drive Control
    public bbbVector2 getDriveControl() {
        double throttle = bbbDoubleUtils.normalize(driveStick.getLeftY());
        double turn = bbbDoubleUtils.normalize(-driveStick.getRightX());

        bbbVector2 control = new bbbVector2(turn, throttle);
        control = JoystickHelper.ScaledAxialDeadzone(control);
        control = JoystickHelper.alternateScaleStick(control, 2);
        control = JoystickHelper.clampStick(control);

        return control;
    }

    // Intake Control
    public double getIntakeControl(){
        return bbbDoubleUtils.normalize(controlStick.getRightTrigger());
    }

    /**
     * Misc
     */
    // Return Auto Command
    public Command getAutonomousCommand() {
        return null;
    }
}
