package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSystem;

public class ControlPanelCommand extends CommandBase {
    private final ControlPanelSystem m_subsystem;

    private final DoubleSupplier control;

    public ControlPanelCommand(ControlPanelSystem controlPanelSystem, DoubleSupplier control) {
        this.m_subsystem = controlPanelSystem;

        this.control = control;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        m_subsystem.setOutput(control.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }
}