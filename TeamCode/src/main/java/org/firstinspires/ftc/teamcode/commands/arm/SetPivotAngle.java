package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class SetPivotAngle extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    private final double angle;

    public SetPivotAngle(ArmSubsystem armSubsystem, double angle) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setPivotAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
