package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class IncrementWristPosition extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    private final double offset;

    public IncrementWristPosition(ArmSubsystem armSubsystem, double offset) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.offset = offset;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setWristPosition(m_armSubsystem.getWristPosition() + offset);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
