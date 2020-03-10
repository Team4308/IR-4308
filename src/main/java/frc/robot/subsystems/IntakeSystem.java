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
    public TalonSRX rollerMotor;

    public int isFlipped;

    public IntakeSystem() {
        intakeMotor = new TalonSRX(Constants.Mapping.Intake.intakeMotor);
        intakeMotor.configFactoryDefault(Constants.Generic.timeoutMs);
        intakeMotor.configOpenloopRamp(Constants.Config.Intake.kOpenLoopRamp, Constants.Generic.timeoutMs);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configContinuousCurrentLimit(15);
        intakeMotor.enableCurrentLimit(true);
        intakeMotor.setInverted(true);

        conveyorMotor = new TalonSRX(Constants.Mapping.Intake.conveyorMotor);
        conveyorMotor.configFactoryDefault(Constants.Generic.timeoutMs);
        conveyorMotor.configOpenloopRamp(Constants.Config.Intake.kOpenLoopRamp, Constants.Generic.timeoutMs);
        conveyorMotor.setNeutralMode(NeutralMode.Brake);
        conveyorMotor.configPeakOutputForward(0.6);
        conveyorMotor.configPeakOutputReverse(-0.6);
        conveyorMotor.follow(intakeMotor);

        rollerMotor = new TalonSRX(Constants.Mapping.Intake.topRoller);
        rollerMotor.configFactoryDefault(Constants.Generic.timeoutMs);
        rollerMotor.configOpenloopRamp(Constants.Config.Intake.kOpenLoopRamp, Constants.Generic.timeoutMs);
        rollerMotor.setNeutralMode(NeutralMode.Brake);
        rollerMotor.follow(intakeMotor);

        isFlipped = 1;
    }

    @Override
    public void stopControllers() {
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