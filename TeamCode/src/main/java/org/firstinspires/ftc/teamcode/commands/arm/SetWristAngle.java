package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class SetWristAngle extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    private final double angle;

    public SetWristAngle(ArmSubsystem armSubsystem, double angle) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setWristAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
