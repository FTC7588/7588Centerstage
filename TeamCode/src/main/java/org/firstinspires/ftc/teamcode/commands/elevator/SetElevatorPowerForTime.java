package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class SetElevatorPowerForTime extends CommandBase {

    public ElevatorSubsystem m_elevatorSubsystem;

    public ElapsedTime timer;

    public double time;
    public double power;

    public SetElevatorPowerForTime(ElevatorSubsystem elevatorSubsystem, double power, double time) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);
        this.power = power;
        this.time = time;
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setPower(power);
        timer.reset();
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > time;
    }
}
