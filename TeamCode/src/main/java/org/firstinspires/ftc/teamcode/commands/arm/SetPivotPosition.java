package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetPivotPosition extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    private final double position;

    public SetPivotPosition(ArmSubsystem armSubsystem, double position) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.position = position;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setPivotPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
