package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import bbb.math.bbbVector2;
import bbb.utils.bbbDoubleUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.enums.DrivetrainMode;
import frc.robot.subsystems.TalonFXDriveSystem;

public class VelocityArcadeDriveCommand extends CommandBase {

    private final TalonFXDriveSystem m_subsystem;
    private final Supplier<bbbVector2> control;

    // INIT
    public VelocityArcadeDriveCommand(TalonFXDriveSystem subsystem, Supplier<bbbVector2> control) {
        this.m_subsystem = subsystem;
        this.control = control;

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.stopControllers();
        m_subsystem.driveMode = DrivetrainMode.VELOCITY;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        bbbVector2 control = this.control.get();
        
        double leftTargetRPM = control.y * Constants.DynConfig.Drive.VelocityDriveRPM;
        double rightTargetRPM = control.y * Constants.DynConfig.Drive.VelocityDriveRPM;

        double newSetpoint = m_subsystem.turnController.getSetpoint() + (control.x * Constants.DynConfig.Drive.GyroTurnSpeed);
        if (newSetpoint > 180) {
            newSetpoint -= 360;
        } else if (newSetpoint < -180) {
            newSetpoint += 360;
        }

        m_subsystem.turnController.setSetpoint(newSetpoint);

        double calculatedTurn = m_subsystem.turnController.calculate(m_subsystem.ahrs.getAngle());

        leftTargetRPM += -bbbDoubleUtils.clamp(calculatedTurn, -1, 1) * Constants.DynConfig.Drive.VelocityDriveRPM;
        rightTargetRPM += bbbDoubleUtils.clamp(calculatedTurn, -1, 1) * Constants.DynConfig.Drive.VelocityDriveRPM;

        leftTargetRPM = bbbDoubleUtils.clamp(leftTargetRPM, -Constants.DynConfig.Drive.VelocityDriveRPM, Constants.DynConfig.Drive.VelocityDriveRPM);
        rightTargetRPM = bbbDoubleUtils.clamp(rightTargetRPM, -Constants.DynConfig.Drive.VelocityDriveRPM, Constants.DynConfig.Drive.VelocityDriveRPM);
        
        double leftTargetUnitsPS = (leftTargetRPM / 600.0) * (Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation);
        double rightTargetUnitsPS = (rightTargetRPM / 600.0) * (Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation);

        m_subsystem.masterLeft.set(TalonFXControlMode.Velocity, leftTargetUnitsPS);
        m_subsystem.masterRight.set(TalonFXControlMode.Velocity, rightTargetUnitsPS);
    }

}