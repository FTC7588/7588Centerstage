package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class SetIntakePower extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;

    private final double intakePower;

    public SetIntakePower(IntakeSubsystem intakeSubsystem, double power) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
        intakePower = power;
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setPower(intakePower);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
