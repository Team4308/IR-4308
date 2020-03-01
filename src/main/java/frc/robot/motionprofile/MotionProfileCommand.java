package frc.robot.motionprofile;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TalonFXDriveSystem;

public class MotionProfileCommand extends CommandBase {
    protected MotionStream profileLeft;
    protected MotionStream profileRight;

    private TalonFXDriveSystem m_subsystem;

    public MotionProfileCommand(String profileFileName, TalonFXDriveSystem subsystem) {
        profileLeft = new MotionStream(profileFileName + "_left");
        profileRight = new MotionStream(profileFileName + "_right");
        this.m_subsystem = subsystem;

        addRequirements(this.m_subsystem);
    }

    @Override
    public void initialize() {
        this.m_subsystem.masterLeft.selectProfileSlot(Constants.Config.Drive.MotionProfile.profileSlot, 0);
        this.m_subsystem.masterRight.selectProfileSlot(Constants.Config.Drive.MotionProfile.profileSlot, 0);
        this.m_subsystem.masterLeft.startMotionProfile(profileLeft.getStream(), 10, ControlMode.MotionProfile);
        this.m_subsystem.masterRight.startMotionProfile(profileRight.getStream(), 10, ControlMode.MotionProfile);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_subsystem.masterLeft.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
        this.m_subsystem.masterRight.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
    }

    @Override
    public boolean isFinished() {
        return this.m_subsystem.masterLeft.isMotionProfileFinished() && this.m_subsystem.masterRight.isMotionProfileFinished();
    }
}