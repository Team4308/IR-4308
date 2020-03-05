package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import bbb.math.bbbVector2;
import bbb.utils.bbbDoubleUtils;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.enums.DrivetrainMode;
import frc.robot.subsystems.TalonFXDriveSystem;

public class VelocityCurvatureDriveCommand extends CommandBase {

    private final TalonFXDriveSystem m_subsystem;

    private final Supplier<bbbVector2> control;

    private final Supplier<Boolean> isQuickturn;

    // INIT
    public VelocityCurvatureDriveCommand(TalonFXDriveSystem subsystem, Supplier<bbbVector2> control, Supplier<Boolean> isQuickturn) {
        this.m_subsystem = subsystem;

        this.control = control;
        this.isQuickturn = isQuickturn;

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.m_subsystem.masterLeft.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
        this.m_subsystem.masterRight.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
        m_subsystem.stopControllers();

        m_subsystem.driveMode = DrivetrainMode.VELOCITY;
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn){

        // Clamp inputs
        speed = Math.max(-1., Math.min(speed, 1.));
        rotation = Math.max(-1., Math.min(rotation, 1.));
    } 

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        bbbVector2 control = this.control.get();
        boolean isQuickturn = this.isQuickturn.get();

        // Calculate constant-curvature vs rate-of-change based on QuickTurn
        double leftTargetRPM;
        double rightTargetRPM;
    
        if (isQuickturn){
    
            // Rate-of-change calculation
            leftTargetRPM = control.y + control.x;
            rightTargetRPM = control.y - control.x;
        } else {
    
            // Constant-curvature calculation
            leftTargetRPM = control.y + Math.abs(control.y) * control.x;
            rightTargetRPM = control.y - Math.abs(control.y) * control.x;
        }
    
        // Normalize wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftTargetRPM), Math.abs(rightTargetRPM));
        if (maxMagnitude > 1.0) {
            leftTargetRPM /= maxMagnitude;
            rightTargetRPM /= maxMagnitude;
        }

        leftTargetRPM *= Constants.DynConfig.Drive.VelocityDriveRPM;
        rightTargetRPM *= Constants.DynConfig.Drive.VelocityDriveRPM;

        leftTargetRPM = bbbDoubleUtils.clamp(leftTargetRPM, -Constants.DynConfig.Drive.VelocityDriveRPM,
                Constants.DynConfig.Drive.VelocityDriveRPM);
        rightTargetRPM = bbbDoubleUtils.clamp(rightTargetRPM, -Constants.DynConfig.Drive.VelocityDriveRPM,
                Constants.DynConfig.Drive.VelocityDriveRPM);

        double leftTargetUnitsPS = (leftTargetRPM / 600.0)
                * (Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation);
        double rightTargetUnitsPS = (rightTargetRPM / 600.0)
                * (Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation);

        m_subsystem.masterLeft.set(TalonFXControlMode.Velocity,
                leftTargetUnitsPS / (12.5 / RobotController.getBatteryVoltage()));
        m_subsystem.masterRight.set(TalonFXControlMode.Velocity,
                rightTargetUnitsPS / (12.5 / RobotController.getBatteryVoltage()));
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.turnController.setSetpoint(m_subsystem.ahrs.getYaw());
    }

}