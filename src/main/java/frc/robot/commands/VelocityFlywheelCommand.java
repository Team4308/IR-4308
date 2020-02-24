package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import bbb.math.bbbVector2;
import bbb.utils.bbbDoubleUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FlywheelSystem;

public class VelocityFlywheelCommand extends CommandBase {
    private final FlywheelSystem m_subsystem;
    Supplier<bbbVector2> control;

    public VelocityFlywheelCommand(FlywheelSystem subsystem, Supplier<bbbVector2> control) {
        this.m_subsystem = subsystem;
        this.control = control;

        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.stopMoving();
    }

    @Override
    public void execute() {
        bbbVector2 control = this.control.get();

        double RPM = control.y * Constants.DynConfig.Flywheel.RPM;
        RPM = bbbDoubleUtils.clamp(RPM, -Constants.DynConfig.Flywheel.RPM, Constants.DynConfig.Flywheel.RPM);
        double targetUnitsPS = (RPM / 600.0) * (Constants.Config.Flywheel.VelocityControl.kSensorUnitsPerRotation);

        m_subsystem.flywheelMotor.set(ControlMode.Velocity, targetUnitsPS);
    }

}