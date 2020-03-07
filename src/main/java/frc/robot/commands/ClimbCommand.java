package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSystem;

public class ClimbCommand extends CommandBase {
    public final ClimbSystem m_subsystem;
    private final DoubleSupplier control;

    public ClimbCommand(ClimbSystem subsystem, DoubleSupplier control) {
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
        m_subsystem.motorControl(control);
    }



}
