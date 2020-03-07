package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Constants;

public class ClimbSystem extends LogSubsystem {
    public VictorSPX master;
    private VictorSPX slave1, slave2, slave3;
    
    private ArrayList<VictorSPX> slaves = new ArrayList<VictorSPX>();

    public ClimbSystem() {
        master = new VictorSPX(Constants.Mapping.Climb.master);

        slave1 = new VictorSPX(Constants.Mapping.Climb.slave1);
        slaves.add(slave1);
        slave2 = new VictorSPX(Constants.Mapping.Climb.slave2);
        slaves.add(slave2);
        slave3 = new VictorSPX(Constants.Mapping.Climb.slave3);
        slaves.add(slave3);

        master.configFactoryDefault();
        master.setNeutralMode(NeutralMode.Brake);

        for (VictorSPX slave : slaves) {
            slave.configFactoryDefault();
            slave.setNeutralMode(NeutralMode.Brake);
            slave.follow(master);
        }

        slave1.setInverted(InvertType.OpposeMaster);
        slave2.setInverted(InvertType.OpposeMaster);
        slave3.setInverted(InvertType.FollowMaster);
    }

    public void motorControl(double control) {
        master.set(ControlMode.PercentOutput, control);
    }

    @Override
    public void stopControllers() {
        master.neutralOutput();
    }

    @Override
    public Sendable log() {
        return this;
    }
}