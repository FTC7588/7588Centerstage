package org.firstinspires.ftc.teamcode.commands.intake;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IncrementIntakeAngle extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;

    private final double increment;

    public IncrementIntakeAngle(IntakeSubsystem intakeSubsystem, double increment) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
        this.increment = increment;
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setPosition(m_intakeSubsystem.getPosition() + increment);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}