package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSystem;

public class RollerCommand extends CommandBase {
    public final RollerSystem m_subsystem;
    private final DoubleSupplier control;

    public RollerCommand(RollerSystem subsystem, DoubleSupplier control) {
        this.m_subsystem = subsystem;
        this.control = control;

        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.stopControllers();
    }

    @Override
    public void execute() {
        double control = this.control.getAsDouble();
        if (!m_subsystem.ballSwitch.get()) {
            m_subsystem.motorControl(control);
        } else {
            m_subsystem.stopControllers();
        }
    }



}
