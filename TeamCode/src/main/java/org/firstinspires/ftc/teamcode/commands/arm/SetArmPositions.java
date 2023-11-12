package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetArmPositions extends SequentialCommandGroup {

    private final ArmSubsystem m_armSubsystem;

    private final double shoulder;
    private final double wrist;
    private final double pivot;

    public SetArmPositions(
            ArmSubsystem armSubsystem,
            double shoulder,
            double wrist,
            double pivot
    ) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.shoulder = shoulder;
        this.wrist = wrist;
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        super.initialize();
        m_armSubsystem.setShoulderPosition(shoulder);
        m_armSubsystem.setWristPosition(wrist);
        m_armSubsystem.setPivotPosition(pivot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
