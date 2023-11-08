package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

public class SetElevatorPower extends CommandBase {

    public ElevatorSubsystem m_elevatorSubsystem;

    public double power;

    public SetElevatorPower(ElevatorSubsystem elevatorSubsystem, double power) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);
        this.power = power;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
