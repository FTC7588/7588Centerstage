package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class LowerElevator extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;

    public LowerElevator(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPower(1);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getAvgCurrent() > 3;
    }
}
