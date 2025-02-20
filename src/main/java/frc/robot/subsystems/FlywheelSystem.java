package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class FlywheelSystem extends LogSubsystem {
    public TalonSRX flywheelMotor;

    public FlywheelSystem() {
        flywheelMotor = new TalonSRX(Constants.Mapping.Flywheel.motor);
        flywheelMotor.configFactoryDefault();
        flywheelMotor.configOpenloopRamp(Constants.Config.Flywheel.kOpenLoopRamp);

        flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.Generic.timeoutMs);
        flywheelMotor.setSensorPhase(false);

        flywheelMotor.config_kF(Constants.Config.Flywheel.VelocityControl.profileSlot,
                Constants.Config.Flywheel.VelocityControl.kF, Constants.Generic.timeoutMs);
        flywheelMotor.config_kP(Constants.Config.Flywheel.VelocityControl.profileSlot,
                Constants.Config.Flywheel.VelocityControl.kP, Constants.Generic.timeoutMs);
        flywheelMotor.config_kI(Constants.Config.Flywheel.VelocityControl.profileSlot,
                Constants.Config.Flywheel.VelocityControl.kI, Constants.Generic.timeoutMs);
        flywheelMotor.config_kD(Constants.Config.Flywheel.VelocityControl.profileSlot,
                Constants.Config.Flywheel.VelocityControl.kD, Constants.Generic.timeoutMs);
        flywheelMotor.selectProfileSlot(Constants.Config.Flywheel.VelocityControl.profileSlot, 0);

        flywheelMotor.configPeakCurrentDuration(0);
        flywheelMotor.configContinuousCurrentLimit(20);
        flywheelMotor.enableCurrentLimit(true);

        stopControllers();
        resetSensors();
    }

    @Override
    public void stopControllers(){
        flywheelMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void resetSensors(){
        flywheelMotor.setSelectedSensorPosition(0, 0, Constants.Generic.timeoutMs);
    }

    @Override
    public Sendable log() {
        return this;
    }

}