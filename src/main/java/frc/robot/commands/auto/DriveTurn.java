package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.enums.DrivetrainMode;
import frc.robot.subsystems.TalonFXDriveSystem;

public class DriveTurn extends CommandBase {
    private TalonFXDriveSystem m_subsystem;
    private double angle = 0.0;

    int withinThresholdLoops = 0;

    public DriveTurn(double angle, TalonFXDriveSystem subsystem) {
        this.angle = angle;
        this.m_subsystem = subsystem;
        
        withinThresholdLoops = 0;

        addRequirements(this.m_subsystem);
    }

    @Override
    public void initialize() {
        this.m_subsystem.resetSensors();
        this.m_subsystem.masterLeft.selectProfileSlot(Constants.Config.Drive.MotionMagic.profileSlot, 0);
        this.m_subsystem.masterRight.selectProfileSlot(Constants.Config.Drive.MotionMagic.profileSlot, 0);
        this.m_subsystem.driveMode = DrivetrainMode.MOTIONMAGIC;
    }

    @Override
    public void execute() {
        double encoderDistance = (22.68 * Math.PI
                / Constants.Config.Drive.Kinematics.kEncoderInchesPerCount);
        encoderDistance /= Constants.Config.Drive.Kinematics.kGearRatio;
        encoderDistance *= this.angle / 360;

        m_subsystem.masterLeft.set(TalonFXControlMode.MotionMagic, encoderDistance);
        m_subsystem.masterRight.set(TalonFXControlMode.MotionMagic, -encoderDistance);

        if (m_subsystem.masterLeft.getActiveTrajectoryPosition() < encoderDistance + 5
                && m_subsystem.masterLeft.getActiveTrajectoryPosition() > encoderDistance - 5
                && m_subsystem.masterRight.getActiveTrajectoryPosition() < -encoderDistance + 5
                && m_subsystem.masterRight.getActiveTrajectoryPosition() > -encoderDistance - 5) {
            withinThresholdLoops += 1;
        } else {
            withinThresholdLoops = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.m_subsystem.masterLeft.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
        this.m_subsystem.masterRight.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
    }

    @Override
    public boolean isFinished() {
        return (withinThresholdLoops > 5);
    }
}