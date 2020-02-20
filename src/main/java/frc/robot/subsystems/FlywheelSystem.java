package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class FlywheelSystem extends LogSubsystem {
    TalonSRX flywheelMotor;

    public FlywheelSystem() {
        flywheelMotor = new TalonSRX(Constants.Mapping.Flywheel.motor);
        flywheelMotor.configFactoryDefault();
        flywheelMotor.configOpenloopRamp(Constants.Config.Flywheel.kOpenLoopRamp);

        flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.Generic.timeoutMs);
        flywheelMotor.setSensorPhase(false);

        flywheelMotor.config_kF(Constants.Config.Flywheel.MotionControl.profileSlot,
                Constants.Config.Flywheel.MotionControl.kF, Constants.Generic.timeoutMs);
        flywheelMotor.config_kP(Constants.Config.Flywheel.MotionControl.profileSlot,
                Constants.Config.Flywheel.MotionControl.kP, Constants.Generic.timeoutMs);
        flywheelMotor.config_kI(Constants.Config.Flywheel.MotionControl.profileSlot,
                Constants.Config.Flywheel.MotionControl.kI, Constants.Generic.timeoutMs);
        flywheelMotor.config_kD(Constants.Config.Flywheel.MotionControl.profileSlot,
                Constants.Config.Flywheel.MotionControl.kD, Constants.Generic.timeoutMs);
        flywheelMotor.selectProfileSlot(Constants.Config.Flywheel.MotionControl.profileSlot, 0);

        stopMoving();
        resetSensors();
    }

    public void stopMoving(){
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