package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sensors.ColorSensor;
import frc.robot.subsystems.ControlPanelSystem;

public class RotatePanelCommand extends CommandBase {

    private final ControlPanelSystem m_subsystem;
    private final ColorSensor m_sensor;
    private boolean finished = false;

    private String firstColorSeen = "";
    private int timesSeen = 0;
    private boolean seen = false;

    public RotatePanelCommand(ControlPanelSystem controlPanelSystem, ColorSensor colorSensor) {
        this.m_subsystem = controlPanelSystem;
        this.m_sensor = colorSensor;

        addRequirements(m_subsystem, m_sensor);
    }

    @Override
    public void initialize() {
        firstColorSeen = m_sensor.getColorDetected();
        m_subsystem.setOutput();
    }

    @Override
    public void execute() {
        if (m_sensor.getColorDetected().equals(firstColorSeen)) {
            if (timesSeen == 7) {
                m_subsystem.stopMoving();
                finished = true;
            }
            seen = true;
        }
        if (seen && !m_sensor.getColorDetected().equals(firstColorSeen)) {
            seen = false;
            timesSeen++;
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