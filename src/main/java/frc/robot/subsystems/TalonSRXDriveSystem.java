package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import bbb.math.bbbVector2;
import bbb.utils.bbbDoubleUtils;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.DrivetrainMode;

public class TalonSRXDriveSystem extends SubsystemBase {
    // Master Controllers
    private final TalonSRX masterLeft, masterRight;
    // Slave Controllers
    private final TalonSRX slaveLeft, slaveRight, slaveLeftM, slaveRightM;

    // AHRS
    private final AHRS ahrs;

    // Controllers
    private ArrayList<TalonSRX> controllers = new ArrayList<TalonSRX>();

    // Current Drive Mode
    private DrivetrainMode driveMode = DrivetrainMode.VELOCITY;

    // Drivetrain Kinematics
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            Constants.Config.Drive.Kinematics.kTrackWidth);

    // Drivetrain Odometry
    private final DifferentialDriveOdometry odometry;

    // Turn PID Controller
    private final PIDController turnController = new PIDController(Constants.Config.Drive.GyroControl.kP,
            Constants.Config.Drive.GyroControl.kI, Constants.Config.Drive.GyroControl.kD);

    // Init
    public TalonSRXDriveSystem(AHRS ahrs) {
        // Setup and Add Controllers
        masterLeft = new TalonSRX(Constants.Mapping.Drive.frontLeft);
        controllers.add(masterLeft);
        masterRight = new TalonSRX(Constants.Mapping.Drive.frontRight);
        controllers.add(masterRight);
        slaveLeft = new TalonSRX(Constants.Mapping.Drive.backLeft);
        controllers.add(slaveLeft);
        slaveRight = new TalonSRX(Constants.Mapping.Drive.backRight);
        controllers.add(slaveRight);
        slaveLeftM = new TalonSRX(Constants.Mapping.Drive.middleLeft);
        controllers.add(slaveLeftM);
        slaveRightM = new TalonSRX(Constants.Mapping.Drive.middleRight);
        controllers.add(slaveRightM);

        // Set AHRS
        this.ahrs = ahrs;

        // Create odometry
        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(ahrs.getAngle()), new Pose2d());

        // Turn PID Controller Config
        turnController.enableContinuousInput(-180, 180);
        turnController.setTolerance(Constants.Config.Drive.GyroControl.kToleranceDegrees);
        turnController.setSetpoint(0.0);

        // Reset Config for all
        for (TalonSRX talon : controllers) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
        }

        // Set Invert Mode
        masterLeft.setInverted(InvertType.None);
        masterRight.setInverted(InvertType.InvertMotorOutput);

        // Set slaves to follow masters
        slaveLeft.follow(masterLeft);
        slaveLeft.setInverted(InvertType.FollowMaster);
        slaveRight.follow(masterRight);
        slaveRight.setInverted(InvertType.FollowMaster);

        // Change Config For All Controllers
        for (TalonSRX talon : controllers) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);

            talon.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp, Constants.Generic.timeoutMs);
        }

        // Configure Primary Closed Loop Sensor
        masterLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.Generic.timeoutMs);
        masterRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,
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

        // Reset
        stopControllers();
        resetSensors();
    }

    // Periodic Loop
    @Override
    public void periodic() {
        // Update odometry
        odometry.update(Rotation2d.fromDegrees(ahrs.getAngle()),
                Units.inchesToMeters(getLeftSensorPosition()), Units.inchesToMeters(getRightSensorPosition()));
    }

    /**
     * Drive functions
     */

    // Normal Arcade Drive
    public void NormalArcadeDrive(bbbVector2 control) {
        masterLeft.set(ControlMode.PercentOutput, control.y, DemandType.ArbitraryFeedForward, control.x);
        masterRight.set(ControlMode.PercentOutput, control.y, DemandType.ArbitraryFeedForward, -control.x);
    }

    // Velocity Arcade Drive
    public void VelocityArcadeDrive(bbbVector2 control) {
        double leftTargetRPM = control.y * Constants.DynConfig.Drive.VelocityDriveRPM;
        double rightTargetRPM = control.y * Constants.DynConfig.Drive.VelocityDriveRPM;

        turnController.setSetpoint(turnController.getSetpoint() + (control.x * Constants.DynConfig.Drive.GyroTurnSpeed));

        leftTargetRPM += bbbDoubleUtils.clamp(turnController.calculate(ahrs.getAngle()), -1, 1) * Constants.DynConfig.Drive.VelocityDriveRPM;
        rightTargetRPM += -bbbDoubleUtils.clamp(turnController.calculate(ahrs.getAngle()), -1, 1) * Constants.DynConfig.Drive.VelocityDriveRPM;

        leftTargetRPM = bbbDoubleUtils.clamp(leftTargetRPM, -Constants.DynConfig.Drive.VelocityDriveRPM, Constants.DynConfig.Drive.VelocityDriveRPM);
        rightTargetRPM = bbbDoubleUtils.clamp(rightTargetRPM, -Constants.DynConfig.Drive.VelocityDriveRPM, Constants.DynConfig.Drive.VelocityDriveRPM);
        
        double leftTargetUnitsPS = leftTargetRPM * Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation / 600.0;
        double rightTargetUnitsPS = rightTargetRPM * Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation / 600.0;

        masterLeft.set(ControlMode.Velocity, leftTargetUnitsPS);
        masterRight.set(ControlMode.Velocity, rightTargetUnitsPS);
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

    /**
     * Misc Stuff
     */

    public void stopControllers() {
        masterLeft.set(ControlMode.PercentOutput, 0.0);
        masterRight.set(ControlMode.PercentOutput, 0.0);
    }

    // Sensor Reset
    public void resetSensors() {
        // Reset Primary Loop
        masterLeft.setSelectedSensorPosition(0, 0, Constants.Generic.timeoutMs);
        masterRight.setSelectedSensorPosition(0, 0, Constants.Generic.timeoutMs);
        turnController.reset();
    }

}