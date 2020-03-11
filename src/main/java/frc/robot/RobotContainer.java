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
import frc.robot.commands.ClimbArmCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ControlPanelCommand;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NormalArcadeDriveCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.VelocityArcadeDriveCommand;
import frc.robot.commands.VelocityCurvatureDriveCommand;
import frc.robot.commands.VelocityFlywheelCommand;
import frc.robot.commands.auto.groups.AngledToPowerPort;
import frc.robot.commands.auto.groups.ForwardToPowerPort;
import frc.robot.commands.auto.groups.RamseteTest;
import frc.robot.commands.auto.groups.Steal;
import frc.robot.commands.auto.groups.TestUltraPathFollower;
import frc.robot.commands.auto.groups.ToPowerPort;
import frc.robot.commands.auto.groups.ToPowerPort1mr;
import frc.robot.subsystems.ClimbArmSystem;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.ControlPanelSystem;
import frc.robot.subsystems.FlywheelSystem;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.RollerSystem;
import frc.robot.subsystems.TalonFXDriveSystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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

    // Control Panel
    private final ControlPanelSystem m_controlPanelSystem;

    // Intake
    private final IntakeSystem m_intakeSystem;

    // Roller
    private final RollerSystem m_rollerSystem;

    // Hopper
    private final HopperSystem m_hopperSystem;

    // Flywheel
    private final FlywheelSystem m_flywheelSystem;

    // Climb
    private final ClimbSystem m_climbSystem;

    // Climb Arm
    private final ClimbArmSystem m_climbArmSystem;

    /**
     * Commands
     */
    // Normal Arcade Drive
    private final NormalArcadeDriveCommand normalArcadeDriveCommand;
    // Velocity Arcade Drive
    private final VelocityArcadeDriveCommand velocityArcadeDriveCommand;
    // Velocity Curveture Drive
    private final VelocityCurvatureDriveCommand velocityCurvatureDriveCommand;

    // Intake
    private final IntakeCommand intakeCommand;

    // Roller
    private final RollerCommand rollerCommand;

    // Hopper
    private final HopperCommand hopperCommand;

    // Flywheel
    private final VelocityFlywheelCommand flywheelCommand;

    // Climb
    private final ClimbCommand climbCommand;

    // Climb Arm
    private final ClimbArmCommand climbArmCommand;

    // Control Panel
    private final ControlPanelCommand controlPanelCommand;

    /**
     * Auto
     */
    private final ToPowerPort toPowerPort;
    private final Steal steal;
    private final ForwardToPowerPort forwardToPowerPort;
    private final AngledToPowerPort angledToPowerPort;
    private final ToPowerPort1mr toPowerPort1mr;
    private final TestUltraPathFollower testUltraPathFollower;
    private final RamseteTest ramseteTest;

    /**
     * Human Controllers
     */
    public final XBoxWrapper driveStick = new XBoxWrapper(0);

    public final XBoxWrapper controlStick = new XBoxWrapper(1);
    private boolean climbMode = false;

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

        m_controlPanelSystem = new ControlPanelSystem();
        subsystems.add(m_controlPanelSystem);

        m_intakeSystem = new IntakeSystem();
        subsystems.add(m_intakeSystem);

        m_rollerSystem = new RollerSystem();
        subsystems.add(m_rollerSystem);

        m_hopperSystem = new HopperSystem();
        subsystems.add(m_hopperSystem);

        m_flywheelSystem = new FlywheelSystem();
        subsystems.add(m_flywheelSystem);

        m_climbSystem = new ClimbSystem();
        subsystems.add(m_climbSystem);

        m_climbArmSystem = new ClimbArmSystem();
        subsystems.add(m_climbArmSystem);

        /**
         * Init Commands
         */
        // Normal Arcade Drive
        normalArcadeDriveCommand = new NormalArcadeDriveCommand(m_driveSystem, () -> getDriveControl());
        // Velocity Arcade Drive
        velocityArcadeDriveCommand = new VelocityArcadeDriveCommand(m_driveSystem, () -> getDriveControl());
        // Velocity Curvature Drive
        velocityCurvatureDriveCommand = new VelocityCurvatureDriveCommand(m_driveSystem, () -> getDriveControl(), () -> driveStick.joystick.getRawButton(6));

        // Intake
        intakeCommand = new IntakeCommand(m_intakeSystem, () -> getIntakeControl());
        m_intakeSystem.setDefaultCommand(intakeCommand);

        // Roller
        rollerCommand = new RollerCommand(m_rollerSystem, () -> getIntakeControl());
        m_rollerSystem.setDefaultCommand(rollerCommand);

        // Hopper
        hopperCommand = new HopperCommand(m_hopperSystem, () -> getIntakeControl() / 4.0);
        m_hopperSystem.setDefaultCommand(hopperCommand);

        // Velocity Flywheel
        flywheelCommand = new VelocityFlywheelCommand(m_flywheelSystem, () -> getFlywheelControl());
        m_flywheelSystem.setDefaultCommand(flywheelCommand);

        // Climb
        climbCommand = new ClimbCommand(m_climbSystem, () -> getClimbControl());
        m_climbSystem.setDefaultCommand(climbCommand);

        // Climb Arm
        climbArmCommand = new ClimbArmCommand(m_climbArmSystem, () -> getClimbArmControl());
        m_climbArmSystem.setDefaultCommand(climbArmCommand);

        // Control Panel
        controlPanelCommand = new ControlPanelCommand(m_controlPanelSystem, () -> getControlPanelControl());
        m_controlPanelSystem.setDefaultCommand(controlPanelCommand);

        /**
         * Init Auto
         */
        toPowerPort = new ToPowerPort(m_driveSystem, m_intakeSystem, m_hopperSystem, m_flywheelSystem);
        steal = new Steal(m_driveSystem, m_intakeSystem);
        forwardToPowerPort = new ForwardToPowerPort(m_driveSystem, m_intakeSystem, m_hopperSystem, m_flywheelSystem);
        angledToPowerPort = new AngledToPowerPort(m_driveSystem, m_intakeSystem, m_hopperSystem, m_flywheelSystem);
        toPowerPort1mr = new ToPowerPort1mr(m_driveSystem, m_intakeSystem, m_hopperSystem, m_flywheelSystem);
        testUltraPathFollower = new TestUltraPathFollower(m_driveSystem);
        ramseteTest = new RamseteTest(m_driveSystem);

        /**
         * Init Choosers
         */
        // Drive Command Chooser
        driveCommandChooser.addOption("Normal Drive", normalArcadeDriveCommand);
        driveCommandChooser.addOption("Curvature Drive", velocityCurvatureDriveCommand);
        driveCommandChooser.setDefaultOption("Velocity Drive", velocityArcadeDriveCommand);
        SmartDashboard.putData(driveCommandChooser);

        // Auto Command Chooser
        autoCommandChooser.addOption("Test Ultra Path Follower", testUltraPathFollower);
        autoCommandChooser.addOption("To Power Port 1 Meter Right", toPowerPort1mr);
        autoCommandChooser.addOption("Forward To Power Port", forwardToPowerPort);
        autoCommandChooser.addOption("Angled To Power Port", angledToPowerPort);
        autoCommandChooser.addOption("Steal", steal);
        autoCommandChooser.addOption("Ramsete Test", ramseteTest);
        autoCommandChooser.setDefaultOption("To Power Port", toPowerPort);
        SmartDashboard.putData(autoCommandChooser);

        /**
         * Configure Button Bindings
         */
        configureButtonBindings();
    }

    // Configure Button Bindings
    private void configureButtonBindings() {
        driveStick.Back.whenPressed(new InstantCommand(() -> m_driveSystem.resetSensors(), m_driveSystem));

        controlStick.LB.whenHeld(new InstantCommand(() -> m_intakeSystem.isFlipped = -1, m_intakeSystem));
        controlStick.LB.whenReleased(new InstantCommand(() -> m_intakeSystem.isFlipped = 1, m_intakeSystem));

        controlStick.RB.whenHeld(new InstantCommand(() -> m_hopperSystem.isFlipped = -1, m_hopperSystem));
        controlStick.RB.whenReleased(new InstantCommand(() -> m_hopperSystem.isFlipped = 1, m_hopperSystem));

        controlStick.Start.whenPressed(new InstantCommand(() -> this.climbMode = false));
        controlStick.Back.whenPressed(new InstantCommand(() -> this.climbMode = true));
    }

    /**
     * Filtered Outputs
     */
    // Drive Control
    public bbbVector2 getDriveControl() {
        if (!climbMode) {
            double throttle = bbbDoubleUtils.normalize(-driveStick.getLeftY());
            double turn = bbbDoubleUtils.normalize(driveStick.getRightX());

            bbbVector2 control = new bbbVector2(turn, throttle);
            control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
            control = JoystickHelper.precisionScaleStick(control, Constants.Config.Input.DriveStick.kInputScale, Constants.Config.Input.DriveStick.kInputPrecision);
            control = JoystickHelper.clampStick(control);

            // if (!JoystickHelper.isStickCentered(control, Constants.Config.Input.kInputDeadband) && !ahrs.isMoving()) {
            //     driveStick.joystick.setRumble(RumbleType.kLeftRumble, 1.0);
            // } else {
            //     driveStick.joystick.setRumble(RumbleType.kLeftRumble, 0.0);
            // }

            return control;
        } else {
            double throttle = bbbDoubleUtils.normalize(-driveStick.getLeftY());
            throttle /= 2.0;
            double turn = bbbDoubleUtils.normalize(driveStick.getRightX());
            turn /= 2.0;

            bbbVector2 control = new bbbVector2(turn, throttle);
            control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
            control = JoystickHelper.precisionScaleStick(control, Constants.Config.Input.DriveStick.kInputScale, Constants.Config.Input.DriveStick.kInputPrecision);
            control = JoystickHelper.clampStick(control);

            double x = bbbDoubleUtils.mapRangeNew(controlStick.getRightTrigger(), 0.0, 1.0, 0.0, 0.25) - bbbDoubleUtils.mapRangeNew(controlStick.getLeftTrigger(), 0.0, 1.0, 0.0, 0.25);
            double y = 0.0;

            if (controlStick.joystick.getRawButton(6)) {
                y += 0.25;
            } else if (controlStick.joystick.getRawButton(5)) {
                y -= 0.25;
            }

            return new bbbVector2(x + control.x, y + control.y);
        }
    }

    // Intake Control
    public double getIntakeControl() {
        if (!climbMode) {
            return bbbDoubleUtils.normalize(controlStick.getRightTrigger());
        }
        return 0.0;
    }

    // Flywheel Control
    public double getFlywheelControl() {
        if (!climbMode) {
            return bbbDoubleUtils.normalize(-controlStick.getLeftTrigger());
        }
        return 0.0;
    }

    // Climb Control
    public double getClimbControl() {
        if (climbMode) {
            double controly = bbbDoubleUtils.normalize(controlStick.getRightY());
            controly = bbbDoubleUtils.clamp(controly, 0.0, 1.0);

            bbbVector2 control = new bbbVector2(0.0, controly);
            control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
            control = JoystickHelper.scaleStick(control, 2.0);
            control = JoystickHelper.clampStick(control);

            return control.y;
        }
        return 0.0;
    }

    public double getClimbArmControl() {
        if (climbMode) {
            double controly = bbbDoubleUtils.normalize(controlStick.getLeftY());
            controly = bbbDoubleUtils.normalize(controly);
            controly /= 2.0;

            bbbVector2 control = new bbbVector2(0.0, controly);
            control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
            control = JoystickHelper.scaleStick(control, 2.0);
            control = JoystickHelper.clampStick(control);

            return control.y;
        }
        return 0.0;
    }

    // Control Panel Control
    public double getControlPanelControl() {
        if (!climbMode) {
            double controlx = bbbDoubleUtils.normalize(controlStick.getLeftX());
            controlx = bbbDoubleUtils.normalize(controlx);

            bbbVector2 control = new bbbVector2(controlx, 0.0);
            control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
            control = JoystickHelper.clampStick(control);

            return control.x;
        }
        return 0.0;
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
