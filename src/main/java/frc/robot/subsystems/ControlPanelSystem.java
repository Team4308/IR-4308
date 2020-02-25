package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class ControlPanelSystem extends LogSubsystem{

    public VictorSPX motor;
    public String colorToGoTo = "";
    
    public ControlPanelSystem() {
        motor = new VictorSPX(Constants.Mapping.ControlPanel.motor);
        motor.configFactoryDefault(Constants.Generic.timeoutMs);
        motor.configOpenloopRamp(Constants.Config.ControlPanel.kOpenLoopRamp, Constants.Generic.timeoutMs);
        stopMoving();
    }

    public void setOutput(){
        motor.set(ControlMode.PercentOutput, Constants.Config.ControlPanel.output);
    }

    public void stopMoving(){
        motor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    colorToGoTo = "B";
                    break;
                case 'G':
                    // Green case code
                    colorToGoTo = "G";
                    break;
                case 'R':
                    // Red case code
                    colorToGoTo = "R";
                    break;
                case 'Y':
                    // Yellow case code
                    colorToGoTo = "Y";
                    break;
                default:
                    // This is corrupt data
                    colorToGoTo = "";
                    break;
            }
        } else {
            // Code for no data received yet
            colorToGoTo = "";
        }
    }

    @Override
    public Sendable log() {
        return this;
    }
}