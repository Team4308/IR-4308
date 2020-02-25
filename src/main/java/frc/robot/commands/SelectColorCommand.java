package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sensors.ColorSensor;
import frc.robot.subsystems.ControlPanelSystem;

public class SelectColorCommand extends CommandBase {

    private final ControlPanelSystem m_subsystem;
    private final ColorSensor m_sensor;

    private boolean finished = false;

    public SelectColorCommand(ControlPanelSystem controlPanelSystem, ColorSensor colorSensor) {
        this.m_subsystem = controlPanelSystem;
        this.m_sensor = colorSensor;

        addRequirements(m_subsystem, m_sensor);
    }

    @Override
    public void initialize() {
        m_subsystem.setOutput();
    }

    @Override
    public void execute() {
        if (m_sensor.getColorDetected().equals(m_subsystem.colorToGoTo)) {
            m_subsystem.stopMoving();
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopMoving();
    }
    
    @Override
    public boolean isFinished() {
        return finished;
    }
}