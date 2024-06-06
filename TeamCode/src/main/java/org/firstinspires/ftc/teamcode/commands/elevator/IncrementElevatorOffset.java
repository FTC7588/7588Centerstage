package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class IncrementElevatorOffset extends CommandBase {

    public ElevatorSubsystem m_elevatorSubsystem;

    public double increment;

    public IncrementElevatorOffset(ElevatorSubsystem elevatorSubsystem, double increment) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);
        this.increment = increment;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.addOffset(increment);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
