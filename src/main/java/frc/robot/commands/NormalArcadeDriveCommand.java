package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import bbb.math.bbbVector2;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.enums.DrivetrainMode;
import frc.robot.subsystems.TalonFXDriveSystem;

public class NormalArcadeDriveCommand extends CommandBase {

    private final TalonFXDriveSystem m_subsystem;
    private final Supplier<bbbVector2> control;

    // INIT
    public NormalArcadeDriveCommand(TalonFXDriveSystem subsystem, Supplier<bbbVector2> control) {
        this.m_subsystem = subsystem;
        this.control = control;

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.stopControllers();
        m_subsystem.driveMode = DrivetrainMode.NORMAL;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        bbbVector2 control = this.control.get();
        m_subsystem.masterLeft.set(TalonFXControlMode.PercentOutput, control.y, DemandType.ArbitraryFeedForward, control.x);
        m_subsystem.masterRight.set(TalonFXControlMode.PercentOutput, control.y, DemandType.ArbitraryFeedForward, -control.x);
    }
}