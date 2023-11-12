package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetShoulderPosition extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    private final double position;

    public SetShoulderPosition(ArmSubsystem armSubsystem, double position) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.position = position;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setShoulderPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
