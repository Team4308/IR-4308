package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sensors.ColorSensor;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelSystem;

public class RotatePanelCommand extends CommandBase {
    
    private final ControlPanelSystem m_subsystem;
    private final ColorSensor m_sensor;
    private boolean finished;

    public RotatePanelCommand(ControlPanelSystem controlPanelSystem, ColorSensor colorSensor) {
        this.m_subsystem = controlPanelSystem;
        this.m_sensor = colorSensor;
        finished = false;
    
        addRequirements(m_subsystem, m_sensor);
    }

    @Override
    public void initialize(){
        Constants.DynConfig.ControlPanel.firstColorSeen = m_sensor.getColorDetected();
        m_subsystem.setOutput();
    }

    @Override
    public void execute(){
        String currentColor = m_sensor.getColorDetected();
        if (currentColor.equals(Constants.DynConfig.ControlPanel.firstColorSeen)){
            if (Constants.DynConfig.ControlPanel.timesSeen == 8){
                m_subsystem.stopMoving();
                finished = true;
            }
            Constants.DynConfig.ControlPanel.seen = true;
        }
        if (Constants.DynConfig.ControlPanel.seen && !currentColor.equals(Constants.DynConfig.ControlPanel.firstColorSeen)){
            Constants.DynConfig.ControlPanel.seen = false;
            Constants.DynConfig.ControlPanel.timesSeen++;
        }
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}