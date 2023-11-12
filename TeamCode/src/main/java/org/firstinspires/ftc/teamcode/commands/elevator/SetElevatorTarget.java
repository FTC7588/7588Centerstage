package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class SetElevatorTarget extends CommandBase {

    public ElevatorSubsystem m_elevatorSubsystem;

    public double target;

    public SetElevatorTarget(ElevatorSubsystem elevatorSubsystem, double target) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);
        this.target = target;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setTarget(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
