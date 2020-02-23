package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class HopperSystem extends LogSubsystem{
    TalonSRX hopperMotor;

    public HopperSystem(){
        hopperMotor = new TalonSRX(Constants.Mapping.Hopper.motor);
        hopperMotor.configFactoryDefault(Constants.Generic.timeoutMs);
        hopperMotor.configOpenloopRamp(Constants.Config.Intake.kOpenLoopRamp, Constants.Generic.timeoutMs);
        hopperMotor.setNeutralMode(NeutralMode.Brake);
        stopMoving();
    }

    public void motorControl(){
        hopperMotor.set(ControlMode.PercentOutput, Constants.Config.Hopper.output);
    }

    public void stopMoving(){
        hopperMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public Sendable log() {
        return this;
    }

}