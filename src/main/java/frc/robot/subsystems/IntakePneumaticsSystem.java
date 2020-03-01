package frc.robot.subsystems;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;
import frc.robot.enums.PneumaticMode;

public class IntakePneumaticsSystem extends LogSubsystem {
    private DoubleSolenoid leftSolenoid, rightSolenoid;
    private PneumaticMode mode;

    public IntakePneumaticsSystem() {
        leftSolenoid = new DoubleSolenoid(Constants.Mapping.PCM_ID, Constants.Mapping.IntakePneumatics.solenoidLeftIn,
                Constants.Mapping.IntakePneumatics.solenoidLeftOut);
        rightSolenoid = new DoubleSolenoid(Constants.Mapping.PCM_ID, Constants.Mapping.IntakePneumatics.solenoidRightIn,
                Constants.Mapping.IntakePneumatics.solenoidRightOut);
    }

    @Override
    public Sendable log() {
        return this;
    }

    public void extend() {
        leftSolenoid.set(DoubleSolenoid.Value.kForward);
        rightSolenoid.set(DoubleSolenoid.Value.kForward);
        mode = PneumaticMode.EXTENDED;
    }

    // The rest doesn't matter
    public void retract() {
        leftSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightSolenoid.set(DoubleSolenoid.Value.kReverse);
        mode = PneumaticMode.RETRACTED;
    }

    public void turnOff() {
        leftSolenoid.set(DoubleSolenoid.Value.kOff);
        rightSolenoid.set(DoubleSolenoid.Value.kOff);
        mode = PneumaticMode.OFF;
    }

}