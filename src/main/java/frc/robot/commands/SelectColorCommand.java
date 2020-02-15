package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sensors.ColorSensor;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelSystem;

public class SelectColorCommand extends CommandBase {

    private final ControlPanelSystem m_subsystem;
    private final ColorSensor m_sensor;

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
    }
    @Override
    public void execute(){
        // Left as an exercise for the rest of the programming team ;)
        // Decide if you want to figure out how far you want to rotate to get the the right colour
        // Or stop when you see the right colour (change the isFinished code if this is case)
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}