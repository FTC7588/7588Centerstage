package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class SetEleArmPositions extends SequentialCommandGroup {

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ArmSubsystem m_armSubsystem;

    private final double elevator;
    private final double shoulder;
    private final double wrist;
    private final double pivot;

    public SetEleArmPositions(
            ElevatorSubsystem elevatorSubsystem,
            ArmSubsystem armSubsystem,
            double elevator,
            double shoulder,
            double wrist,
            double pivot
    ) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        addRequirements(m_elevatorSubsystem, m_armSubsystem);
        this.elevator = elevator;
        this.shoulder = shoulder;
        this.wrist = wrist;
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        super.initialize();
        m_elevatorSubsystem.setTarget(elevator);
        m_armSubsystem.setShoulderPosition(shoulder);
        m_armSubsystem.setWristPosition(wrist);
        m_armSubsystem.setPivotPosition(pivot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
