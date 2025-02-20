package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import bbb.wrapper.TankDriveSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.enums.DrivetrainMode;

public class TalonFXDriveSystem extends TankDriveSubsystem {
    // Master Controllers
    public final TalonFX masterLeft, masterRight;
    // Slave Controllers
    private final TalonFX slaveLeft, slaveRight;

    // Controllers
    private ArrayList<TalonFX> controllers = new ArrayList<TalonFX>();

    // AHRS
    public final AHRS ahrs;

    // Current Drive Mode
    public DrivetrainMode driveMode = DrivetrainMode.VELOCITY;

    // Drivetrain Kinematics
    public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            Constants.Config.Drive.Kinematics.kTrackWidth);

    // Drivetrain Odometry
    public final DifferentialDriveOdometry odometry;

    // Turn PID Controller (Velocity)
    public final PIDController turnController = new PIDController(Constants.Config.Drive.GyroControl.kP,
            Constants.Config.Drive.GyroControl.kI, Constants.Config.Drive.GyroControl.kD);

    // Init
    public TalonFXDriveSystem(AHRS ahrs) {
        // Setup and Add Controllers
        masterLeft = new TalonFX(Constants.Mapping.Drive.frontLeft);
        controllers.add(masterLeft);
        masterRight = new TalonFX(Constants.Mapping.Drive.frontRight);
        controllers.add(masterRight);
        slaveLeft = new TalonFX(Constants.Mapping.Drive.backLeft);
        controllers.add(slaveLeft);
        slaveRight = new TalonFX(Constants.Mapping.Drive.backRight);
        controllers.add(slaveRight);

        // Set AHRS
        this.ahrs = ahrs;

        // Create odometry
        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(ahrs.getAngle()), new Pose2d());

        // Turn PID Controller Config
        turnController.enableContinuousInput(-180, 180);
        turnController.setTolerance(Constants.Config.Drive.GyroControl.kToleranceDegrees / 5.4);
        turnController.setSetpoint(ahrs.getYaw());

        // Reset Config for all
        for (TalonFX talon : controllers) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
        }

        // Set Invert Mode
        masterLeft.setInverted(TalonFXInvertType.CounterClockwise);
        masterRight.setInverted(TalonFXInvertType.Clockwise);

        // Set slaves to follow masters
        slaveLeft.follow(masterLeft);
        slaveLeft.setInverted(TalonFXInvertType.FollowMaster);
        slaveRight.follow(masterRight);
        slaveRight.setInverted(TalonFXInvertType.FollowMaster);

        // Change Config For All Controllers
        for (TalonFX talon : controllers) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);

            talon.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp, Constants.Generic.timeoutMs);
            talon.configClosedloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp, Constants.Generic.timeoutMs);
            talon.configStatorCurrentLimit(Constants.Config.Drive.Power.kStatorCurrentLimit,
                    Constants.Generic.timeoutMs);
            talon.setNeutralMode(NeutralMode.Coast);
            talon.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
            talon.changeMotionControlFramePeriod(5);
            talon.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
            talon.enableVoltageCompensation(true);
        }

        // Configure Primary Closed Loop Sensor
        masterLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.Generic.timeoutMs);
        masterRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.Generic.timeoutMs);

        // Set Sensor Phase for all loops
        masterLeft.setSensorPhase(false);
        masterLeft.setSensorPhase(false);

        // Set Left Velocity PIDF values
        masterLeft.config_kP(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Left.kP, Constants.Generic.timeoutMs);
        masterLeft.config_kI(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Left.kI, Constants.Generic.timeoutMs);
        masterLeft.config_kD(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Left.kD, Constants.Generic.timeoutMs);
        masterLeft.config_kF(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Left.kF, Constants.Generic.timeoutMs);
        masterLeft.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);

        // Set Right Velocity PIDF values
        masterRight.config_kP(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Right.kP, Constants.Generic.timeoutMs);
        masterRight.config_kI(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Right.kI, Constants.Generic.timeoutMs);
        masterRight.config_kD(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Right.kD, Constants.Generic.timeoutMs);
        masterRight.config_kF(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Right.kF, Constants.Generic.timeoutMs);
        masterRight.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);

        // Motion Magic Configuration
        masterLeft.configMotionCruiseVelocity(Constants.Config.Drive.MotionMagic.maxVel);
        masterRight.configMotionCruiseVelocity(Constants.Config.Drive.MotionMagic.maxVel);
        masterLeft.configMotionAcceleration(Constants.Config.Drive.MotionMagic.maxAcc);
        masterRight.configMotionAcceleration(Constants.Config.Drive.MotionMagic.maxAcc);

        masterLeft.config_kP(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kP, Constants.Generic.timeoutMs);
        masterLeft.config_kI(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kI, Constants.Generic.timeoutMs);
        masterLeft.config_kD(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kD, Constants.Generic.timeoutMs);
        masterLeft.config_kF(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kF, Constants.Generic.timeoutMs);

        masterRight.config_kP(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kP, Constants.Generic.timeoutMs);
        masterRight.config_kI(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kI, Constants.Generic.timeoutMs);
        masterRight.config_kD(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kD, Constants.Generic.timeoutMs);
        masterRight.config_kF(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kF, Constants.Generic.timeoutMs);

        // Reset
        stopControllers();
        resetSensors();
    }

    // Periodic Loop
    @Override
    public void periodic() {
        // Update odometry
        odometry.update(Rotation2d.fromDegrees(ahrs.getYaw()),
                Units.inchesToMeters(
                        getLeftSensorPosition()),
                Units.inchesToMeters(
                        getRightSensorPosition()));
    }

    /**
     * Getters And Setters
     */

    public double getLeftSensorPosition() {
        return masterLeft.getSelectedSensorPosition(0) * Constants.Config.Drive.Kinematics.kEncoderInchesPerCount;
    }

    public double getRightSensorPosition() {
        return masterRight.getSelectedSensorPosition(0) * Constants.Config.Drive.Kinematics.kEncoderInchesPerCount;
    }

    public double getLeftSensorVelocity() {
        return masterLeft.getSelectedSensorVelocity(0);
    }

    public double getRightSensorVelocity() {
        return masterRight.getSelectedSensorVelocity(0);
    }

    public void setMotorVelocityMPS(double left, double right) {
        masterLeft.set(TalonFXControlMode.Velocity,
                ((Units.metersToInches(left) / Constants.Config.Drive.Kinematics.kEncoderInchesPerCount)
                        / Constants.Config.Drive.Kinematics.kGearRatio) / 10.0);
        masterRight.set(TalonFXControlMode.Velocity,
                ((Units.metersToInches(right) / Constants.Config.Drive.Kinematics.kEncoderInchesPerCount)
                        / Constants.Config.Drive.Kinematics.kGearRatio) / 10.0);
    }

    /**
     * Misc Stuff
     */

    public void stopControllers() {
        masterLeft.set(TalonFXControlMode.PercentOutput, 0.0);
        masterRight.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    // Sensor Reset
    public void resetSensors() {
        masterLeft.setSelectedSensorPosition(0);
        masterRight.setSelectedSensorPosition(0);
        turnController.reset();
        ahrs.reset();
    }

    @Override
    public Sendable log() {
        Shuffleboard.getTab("Log").addNumber("TurnC SP", () -> turnController.getSetpoint());
        Shuffleboard.getTab("Log").addNumber("Left Vel",
                () -> ((getLeftSensorVelocity() / Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation) * 600));
        Shuffleboard.getTab("Log").addNumber("Right Vel",
                () -> ((getRightSensorVelocity() / Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation) * 600));
        Shuffleboard.getTab("Log").addNumber("Left Pos", () -> getLeftSensorPosition());
        Shuffleboard.getTab("Log").addNumber("Right Pos", () -> getRightSensorPosition());

        return this;
    }

    @Override
    public void setMotorOutput(ControlMode mode, double left, double right) {
        masterLeft.set(mode, left);
        masterRight.set(mode, right);
    }

    @Override
    public AHRS getAhrs() {
        return ahrs;
    }

    @Override
    public void selectProfileSlot(int slot) {
        masterLeft.selectProfileSlot(slot, 0);
        masterRight.selectProfileSlot(slot, 0);
    }

}