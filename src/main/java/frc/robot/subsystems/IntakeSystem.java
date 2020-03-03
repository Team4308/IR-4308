package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class IntakeSystem extends LogSubsystem {
    public TalonSRX intakeMotor;
    public TalonSRX conveyorMotor;
    public TalonSRX hopperMotor;

    public int isFlipped;

    public IntakeSystem() {
        intakeMotor = new TalonSRX(Constants.Mapping.Intake.intakeMotor);
        intakeMotor.configFactoryDefault(Constants.Generic.timeoutMs);
        intakeMotor.configOpenloopRamp(Constants.Config.Intake.kOpenLoopRamp, Constants.Generic.timeoutMs);
        intakeMotor.setNeutralMode(NeutralMode.Coast);

        conveyorMotor = new TalonSRX(Constants.Mapping.Intake.conveyorMotor);
        conveyorMotor.configFactoryDefault(Constants.Generic.timeoutMs);
        conveyorMotor.configOpenloopRamp(Constants.Config.Intake.kOpenLoopRamp, Constants.Generic.timeoutMs);
        conveyorMotor.setNeutralMode(NeutralMode.Coast);
        conveyorMotor.follow(intakeMotor);

        hopperMotor = new TalonSRX(Constants.Mapping.Intake.hopperMotor);
        hopperMotor.configFactoryDefault(Constants.Generic.timeoutMs);
        hopperMotor.configOpenloopRamp(Constants.Config.Intake.kOpenLoopRamp, Constants.Generic.timeoutMs);
        hopperMotor.setNeutralMode(NeutralMode.Coast);
        hopperMotor.follow(intakeMotor);

        isFlipped = 1;
    }

    public void stopMoving() {
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void motorControl(double control) {
        intakeMotor.set(ControlMode.PercentOutput, control * isFlipped);
    }

    @Override
    public Sendable log() {
        return this;
    }
}