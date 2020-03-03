package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sensors.ColorSensor;
import frc.robot.subsystems.ControlPanelSystem;

public class ControlPanelCommand extends CommandBase {
    private final ControlPanelSystem m_subsystem;
    private final ColorSensor m_colorSensor;

    private final DoubleSupplier control;

    public ControlPanelCommand(ControlPanelSystem controlPanelSystem, ColorSensor colorSensor, DoubleSupplier control) {
        this.m_subsystem = controlPanelSystem;
        this.m_colorSensor = colorSensor;

        this.control = control;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        m_subsystem.setOutput(control.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopMoving();
    }
}