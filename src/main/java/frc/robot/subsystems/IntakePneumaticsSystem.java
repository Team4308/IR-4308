package frc.robot.subsystems;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class IntakePneumaticsSystem extends LogSubsystem {
    private DoubleSolenoid leftSolenoid, rightSolenoid;

    public IntakePneumaticsSystem() {
        leftSolenoid = new DoubleSolenoid(Constants.Mapping.PCM_ID, Constants.Mapping.IntakePneumatics.solenoidLeftIn,
                Constants.Mapping.IntakePneumatics.solenoidLeftOut);
        rightSolenoid = new DoubleSolenoid(Constants.Mapping.PCM_ID, Constants.Mapping.IntakePneumatics.solenoidRightIn,
                Constants.Mapping.IntakePneumatics.solenoidRightOut);

        turnOff();
    }

    @Override
    public Sendable log() {
        return this;
    }

    public void extend() {
        leftSolenoid.set(DoubleSolenoid.Value.kForward);
        rightSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    // The rest doesn't matter
    public void retract() {
        leftSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void turnOff() {
        leftSolenoid.set(DoubleSolenoid.Value.kOff);
        rightSolenoid.set(DoubleSolenoid.Value.kOff);
    }

}