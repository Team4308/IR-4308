package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sensors.ColorSensor;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelSystem;

public class SelectColorCommand extends CommandBase {

    private final ControlPanelSystem m_subsystem;
    private final ColorSensor m_sensor;

    private boolean finished = false;

    public SelectColorCommand() {
        m_subsystem = new ControlPanelSystem();
        m_sensor = new ColorSensor();

        addRequirements(m_subsystem, m_sensor);
    }

    @Override
    public void initialize() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    Constants.DynConfig.ControlPanel.gameDataColor = gameData;
                    break;
                case 'G':
                    // Green case code
                    Constants.DynConfig.ControlPanel.gameDataColor = gameData;
                    break;
                case 'R':
                    // Red case code
                    Constants.DynConfig.ControlPanel.gameDataColor = gameData;
                    break;
                case 'Y':
                    // Yellow case code
                    Constants.DynConfig.ControlPanel.gameDataColor = gameData;
                    break;
                default:
                    // This is corrupt data
                    Constants.DynConfig.ControlPanel.gameDataColor = "";
                    break;
            }
        } else {
            // Code for no data received yet
            Constants.DynConfig.ControlPanel.gameDataColor = "";
        }
        Constants.DynConfig.ControlPanel.firstColorSeen = m_sensor.getColorDetected();
        m_subsystem.setOutput();
    }

    @Override
    public void execute() {
        String currentColor = m_sensor.getColorDetected();
        if (currentColor.equals(Constants.DynConfig.ControlPanel.gameDataColor)) {
            m_subsystem.stopMoving();
            finished = true;
        }
    }
    
    @Override
    public boolean isFinished() {
        return finished;
    }
}