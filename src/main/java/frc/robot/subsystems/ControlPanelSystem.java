package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

public class ControlPanelSystem extends LogSubsystem {

    public TalonSRX motor;
    public String colorToGoTo = "";

    public ControlPanelSystem() {
        motor = new TalonSRX(Constants.Mapping.ControlPanel.motor);
        motor.configFactoryDefault(Constants.Generic.timeoutMs);
        motor.configOpenloopRamp(Constants.Config.ControlPanel.kOpenLoopRamp, Constants.Generic.timeoutMs);
        motor.setNeutralMode(NeutralMode.Brake);
        stopControllers();
    }

    public void setOutput(double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public void stopControllers() {
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
        Shuffleboard.getTab("Log").addString("Color To Go To", () -> colorToGoTo);
        return this;
    }
}