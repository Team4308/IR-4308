package bbb.wrapper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

public abstract class TankDriveSubsystem extends LogSubsystem {
    public abstract void setMotorOutput(ControlMode mode, double left, double right);

    public abstract void selectProfileSlot(int slot);
    public abstract void resetSensors();

    public abstract double getLeftSensorPosition();
    public abstract double getRightSensorPosition();
    public abstract double getLeftSensorVelocity();
    public abstract double getRightSensorVelocity();

    public abstract AHRS getAhrs();
}