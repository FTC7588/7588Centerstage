package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetShoulderAngle extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    private final double angle;

    public SetShoulderAngle(ArmSubsystem armSubsystem, double angle) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setShoulderAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
