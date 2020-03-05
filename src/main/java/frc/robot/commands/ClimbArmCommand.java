package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArmSystem;

public class ClimbArmCommand extends CommandBase {
    private ClimbArmSystem m_subsystem;

    private DoubleSupplier control;

    public ClimbArmCommand(ClimbArmSystem climbArmSystem, DoubleSupplier control) {
        this.m_subsystem = climbArmSystem;
        
        this.control = control;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        this.m_subsystem.setOutput(control.getAsDouble());
    }
}