package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class LowerElevator extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;

    private ElapsedTime timer;

    public LowerElevator(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPower(1);
        timer.reset();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getAvgCurrent() > 3.25;
    }
}
