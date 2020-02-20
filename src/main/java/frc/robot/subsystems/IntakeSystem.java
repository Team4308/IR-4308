package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class IntakeSystem extends LogSubsystem {
    public TalonSRX intakeMotor;
    public int isFlipped;
    
    public IntakeSystem(){
        intakeMotor = new TalonSRX(Constants.Mapping.Intake.motor);
        intakeMotor.configFactoryDefault(Constants.Generic.timeoutMs);
        intakeMotor.configOpenloopRamp(Constants.Config.Intake.kOpenLoopRamp, Constants.Generic.timeoutMs);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        isFlipped = 1;
    }

    public void stopMoving(){
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