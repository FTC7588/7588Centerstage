package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetPivotPositions extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    private final ArmSubsystem.PivotRotatedState rotation;
    private final ArmSubsystem.PivotPositionState position;

    public SetPivotPositions(ArmSubsystem armSubsystem, ArmSubsystem.PivotRotatedState rotation, ArmSubsystem.PivotPositionState position) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.position = position;
        this.rotation = rotation;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setPivotStates(rotation, position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
