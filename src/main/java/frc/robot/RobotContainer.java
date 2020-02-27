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
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NormalArcadeDriveCommand;
import frc.robot.commands.RotatePanelCommand;
import frc.robot.commands.SelectColorCommand;
import frc.robot.commands.VelocityArcadeDriveCommand;
import frc.robot.commands.VelocityFlywheelCommand;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveTurn;
import frc.robot.commands.auto.groups.TestAuto;
import frc.robot.commands.auto.groups.TestMotionProfile;
import frc.robot.subsystems.sensors.ColorSensor;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.ControlPanelSystem;
import frc.robot.subsystems.FlywheelSystem;
import frc.robot.subsystems.HopperSystem;
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
    public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

    // Drivetrain
    private final TalonFXDriveSystem m_driveSystem;

    // Color Sensor
    private final ColorSensor m_colorSensor;

    // Control Panel
    private final ControlPanelSystem m_controlPanelSystem;

    // Intake
    private final IntakeSystem m_intakeSystem;

    // Flywheel
    private final FlywheelSystem m_flywheelSystem;

    // Hopper
    private final HopperSystem m_hopperSystem;

    // Climb
    private final ClimbSystem m_climbSystem;

    /**
     * Commands
     */
    // Normal Arcade Drive
    private final NormalArcadeDriveCommand normalArcadeDriveCommand;
    // Velocity Arcade Drive
    private final VelocityArcadeDriveCommand velocityArcadeDriveCommand;

    // Intake
    private final IntakeCommand intakeCommand;

    // Flywheel
    private final VelocityFlywheelCommand flywheelCommand;

    // Climb
    private final ClimbCommand climbCommand;

    /**
     * Auto
     */
    private final TestAuto testAuto;
    private final TestMotionProfile testMotionProfile;

    /**
     * Human Controllers
     */
    public final XBoxWrapper driveStick = new XBoxWrapper(0);
    public final XBoxWrapper controlStick = new XBoxWrapper(1);

    /**
     * Choosers
     */
    // Drive Chooser
    private final SendableChooser<Command> driveCommandChooser = new SendableChooser<Command>();

    // Auto Chooser
    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();
    

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

        m_flywheelSystem = new FlywheelSystem();
        subsystems.add(m_flywheelSystem);

        m_hopperSystem = new HopperSystem();
        subsystems.add(m_hopperSystem);

        m_climbSystem = new ClimbSystem();
        subsystems.add(m_climbSystem);

        /**
         * Init Commands
         */
        // Normal Arcade Drive
        normalArcadeDriveCommand = new NormalArcadeDriveCommand(m_driveSystem, () -> getDriveControl());
        // Velocity Arcade Drive
        velocityArcadeDriveCommand = new VelocityArcadeDriveCommand(m_driveSystem, () -> getDriveControl());

        // Intake
        intakeCommand = new IntakeCommand(m_intakeSystem, () -> getIntakeControl());
        m_intakeSystem.setDefaultCommand(intakeCommand);

        // Velocity Flywheel
        flywheelCommand = new VelocityFlywheelCommand(m_flywheelSystem, () -> getFlywheelControl());
        m_flywheelSystem.setDefaultCommand(flywheelCommand);

        // Climb
        climbCommand = new ClimbCommand(m_climbSystem, () -> getClimbControl());
        m_climbSystem.setDefaultCommand(climbCommand);

        /**
         * Init Auto
         */
        testAuto = new TestAuto(m_driveSystem);
        testMotionProfile = new TestMotionProfile(m_driveSystem);

        /**
         * Init Choosers
         */
        // Drive Command Chooser
        driveCommandChooser.addOption("Normal Drive", normalArcadeDriveCommand);
        driveCommandChooser.setDefaultOption("Velocity Drive", velocityArcadeDriveCommand);
        SmartDashboard.putData(driveCommandChooser);

        // Auto Command Chooser
        autoCommandChooser.addOption("Test Motion Profile", testMotionProfile);
        autoCommandChooser.setDefaultOption("Test Auto", testAuto);
        SmartDashboard.putData(autoCommandChooser);

        /**
         * Configure Button Bindings
         */
        configureButtonBindings();
    }

    // Configure Button Bindings
    private void configureButtonBindings() {
        driveStick.Back.whenPressed(new InstantCommand(() -> m_driveSystem.resetSensors(), m_driveSystem));
        controlStick.Back.whenPressed(new RotatePanelCommand(m_controlPanelSystem, m_colorSensor));
        controlStick.Start.whenPressed(new SelectColorCommand(m_controlPanelSystem, m_colorSensor));
        controlStick.RB.whenPressed(new InstantCommand(() -> m_intakeSystem.isFlipped = 1, m_intakeSystem));
        controlStick.LB.whenPressed(new InstantCommand(() -> m_intakeSystem.isFlipped = -1, m_intakeSystem));
        controlStick.A.whenPressed(new InstantCommand(() -> m_hopperSystem.motorControl(), m_hopperSystem));
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
    public double getIntakeControl() {
        return bbbDoubleUtils.normalize(controlStick.getRightTrigger());
    }

    // Flywheel Control
    public double getFlywheelControl() {
        return bbbDoubleUtils.normalize(controlStick.getLeftTrigger());
    }

    // Climb Control
    public double getClimbControl() {
        double controly = bbbDoubleUtils.normalize(controlStick.getRightY());
        controly = bbbDoubleUtils.clamp(controly, -1.0, 0.0);

        bbbVector2 control = new bbbVector2(0.0, controly);
        control = JoystickHelper.ScaledAxialDeadzone(control);
        control = JoystickHelper.clampStick(control);

        return control.y;
    }

    /**
     * Misc
     */
    // Return Teleop Command
    public Command getTeleopCommand() {
        return driveCommandChooser.getSelected();
    }

    // Return Auto Command
    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected();
    }
}
