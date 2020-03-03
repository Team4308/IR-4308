package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class ClimbArmSystem extends LogSubsystem {
    public TalonSRX motor;

    public ClimbArmSystem() {
        motor = new TalonSRX(Constants.Mapping.ClimbArm.motor);
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void setOutput(double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public Sendable log() {
        return this;
    }
    
}