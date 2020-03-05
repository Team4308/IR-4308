package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSystem;

public class HopperCommand extends CommandBase {
    public final HopperSystem m_subsystem;
    private final DoubleSupplier control;

    public HopperCommand(HopperSystem subsystem, DoubleSupplier control) {
        this.m_subsystem = subsystem;
        this.control = control;

        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.stopMoving();
    }

    @Override
    public void execute() {
        double control = this.control.getAsDouble();
        m_subsystem.motorControl(control);
    }



}
