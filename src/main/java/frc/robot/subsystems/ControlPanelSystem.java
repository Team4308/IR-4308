package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class ControlPanelSystem extends LogSubsystem{

    public TalonSRX motor;
    
    public ControlPanelSystem(){
        motor = new TalonSRX(Constants.Mapping.ControlPanel.motor);
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
    public Sendable log() {
        return this;
    }
}