package org.firstinspires.ftc.teamcode.poofyutils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import java.util.ArrayList;
import java.util.List;

public class RunDuringDelayCommand extends CommandBase {

    private Command firstCmd;
    private double msDelay;
    private Command secondCmd;

    public RunDuringDelayCommand(Command firstCmd, double msDelay, Command secondCmd) {
        this.firstCmd = firstCmd;
        this.msDelay = msDelay;
        this.secondCmd = secondCmd;
    }

    @Override
    public void initialize() {
        super.initialize();

        new ParallelCommandGroup(
                firstCmd,
                new SequentialCommandGroup(
                        new WaitCommand((long) msDelay),
                        secondCmd
                )
        ).schedule();
    }

    @Override
    public boolean isFinished() {
        return (firstCmd.isFinished() && secondCmd.isFinished());
    }
}
