package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class SetIntakeAngle extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;

    private final double position;

    public SetIntakeAngle(IntakeSubsystem intakeSubsystem, double pos) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
        this.position = pos;
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
