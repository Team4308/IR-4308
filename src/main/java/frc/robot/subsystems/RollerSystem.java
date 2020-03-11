package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class RollerSystem extends LogSubsystem {
    public TalonSRX rollerMotor;
    public DigitalInput ballSwitch;

    public int isFlipped;

    public RollerSystem() {
        rollerMotor = new TalonSRX(Constants.Mapping.Intake.topRoller);
        rollerMotor.configFactoryDefault(Constants.Generic.timeoutMs);
        rollerMotor.configOpenloopRamp(Constants.Config.Intake.kOpenLoopRamp, Constants.Generic.timeoutMs);
        rollerMotor.setNeutralMode(NeutralMode.Brake);

        ballSwitch = new DigitalInput(Constants.Mapping.Intake.ballSwitch);

        isFlipped = 1;
    }

    @Override
    public void stopControllers() {
        rollerMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void motorControl(double control) {
        rollerMotor.set(ControlMode.PercentOutput, control * isFlipped);
    }

    @Override
    public Sendable log() {
        return this;
    }
}