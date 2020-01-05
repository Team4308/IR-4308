package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSystem extends SubsystemBase {
    // Master Controllers
    private TalonFX masterLeft, masterRight;
    // Slave Controllers
    private TalonFX slaveLeft, slaveRight;

    // Controllers
    private ArrayList<TalonFX> controllers = new ArrayList<TalonFX>();

    // Drive RPM for Velocity Control
    private double driveRPM = 250;

    // Init
    public DriveSystem() {
        // Setup and Add Controllers
        masterLeft = new TalonFX(Constants.Mapping.Drive.frontLeft);
        controllers.add(masterLeft);
        masterRight = new TalonFX(Constants.Mapping.Drive.frontRight);
        controllers.add(masterRight);
        slaveLeft = new TalonFX(Constants.Mapping.Drive.backLeft);
        controllers.add(slaveLeft);
        slaveRight = new TalonFX(Constants.Mapping.Drive.backRight);
        controllers.add(slaveRight);

        // Change Config For All Controllers
        for (TalonFX talon : controllers) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);

            talon.configStatorCurrentLimit(Constants.Config.Drive.Power.kStatorCurrentLimit,
                    Constants.Generic.timeoutMs);
        }

        // Set Invert Mode
        masterLeft.setInverted(TalonFXInvertType.Clockwise);
        masterRight.setInverted(TalonFXInvertType.CounterClockwise);

        // Set slaves to follow masters
        slaveLeft.follow(masterLeft);
        slaveLeft.setInverted(TalonFXInvertType.FollowMaster);
        slaveRight.follow(masterRight);
        slaveRight.setInverted(TalonFXInvertType.FollowMaster);

        // Configure Primary Closed Loop Sensor
        masterLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.Generic.timeoutMs);
        masterRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.Generic.timeoutMs);

        // Set Sensor Phase for all loops
        masterLeft.setSensorPhase(false);
        masterLeft.setSensorPhase(false);

        // Set Left PIDF values
        masterLeft.config_kP(Constants.Config.Drive.MotionControl.profileSlot,
                Constants.Config.Drive.MotionControl.Left.kP, Constants.Generic.timeoutMs);
        masterLeft.config_kI(Constants.Config.Drive.MotionControl.profileSlot,
                Constants.Config.Drive.MotionControl.Left.kI, Constants.Generic.timeoutMs);
        masterLeft.config_kD(Constants.Config.Drive.MotionControl.profileSlot,
                Constants.Config.Drive.MotionControl.Left.kD, Constants.Generic.timeoutMs);
        masterLeft.config_kF(Constants.Config.Drive.MotionControl.profileSlot,
                Constants.Config.Drive.MotionControl.Left.kF, Constants.Generic.timeoutMs);
        masterLeft.selectProfileSlot(Constants.Config.Drive.MotionControl.profileSlot, 0);

        // Set Right PIDF values
        masterRight.config_kP(Constants.Config.Drive.MotionControl.profileSlot,
                Constants.Config.Drive.MotionControl.Right.kP, Constants.Generic.timeoutMs);
        masterRight.config_kI(Constants.Config.Drive.MotionControl.profileSlot,
                Constants.Config.Drive.MotionControl.Right.kI, Constants.Generic.timeoutMs);
        masterRight.config_kD(Constants.Config.Drive.MotionControl.profileSlot,
                Constants.Config.Drive.MotionControl.Right.kD, Constants.Generic.timeoutMs);
        masterRight.config_kF(Constants.Config.Drive.MotionControl.profileSlot,
                Constants.Config.Drive.MotionControl.Right.kF, Constants.Generic.timeoutMs);
        masterRight.selectProfileSlot(Constants.Config.Drive.MotionControl.profileSlot, 0);
    }

    // Periodic Loop
    @Override
    public void periodic() {
    }

    // Sensor Reset
    public void resetSensors() {
        // Reset Primary Loop
        masterLeft.setSelectedSensorPosition(0, 0, Constants.Generic.timeoutMs);
        masterRight.setSelectedSensorPosition(0, 0, Constants.Generic.timeoutMs);
    }

}