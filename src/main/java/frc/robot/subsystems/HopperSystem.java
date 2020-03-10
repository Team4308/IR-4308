package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class HopperSystem extends LogSubsystem {
    public TalonSRX hopperMotor;

    public int isFlipped;

    public HopperSystem() {
        hopperMotor = new TalonSRX(Constants.Mapping.Intake.hopperMotor);
        hopperMotor.configFactoryDefault(Constants.Generic.timeoutMs);
        hopperMotor.configOpenloopRamp(Constants.Config.Intake.kOpenLoopRamp, Constants.Generic.timeoutMs);
        hopperMotor.configPeakOutputForward(0.25);
        hopperMotor.configPeakOutputReverse(-0.25);
        hopperMotor.setNeutralMode(NeutralMode.Coast);
        hopperMotor.setInverted(true);

        isFlipped = 1;
    }

    @Override
    public void stopControllers() {
        hopperMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void motorControl(double control) {
        hopperMotor.set(ControlMode.PercentOutput, control * isFlipped);
    }

    @Override
    public Sendable log() {
        return this;
    }
}