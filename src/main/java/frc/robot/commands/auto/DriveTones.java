package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TalonFXDriveSystem;

public class DriveTones extends CommandBase {
    private TalonFXDriveSystem m_subsystem;

    private boolean isFinished = false;

    public DriveTones(TalonFXDriveSystem subsystem) {
        this.m_subsystem = subsystem;

        addRequirements(this.m_subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}