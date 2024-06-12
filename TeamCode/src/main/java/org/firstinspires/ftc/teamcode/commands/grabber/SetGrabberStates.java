package org.firstinspires.ftc.teamcode.commands.grabber;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GrabberSubsystem;

public class SetGrabberStates extends CommandBase {

    private GrabberSubsystem grabberSubsystem;

    private GrabberSubsystem.GrabberState state;

    public SetGrabberStates(GrabberSubsystem grabberSubsystem, GrabberSubsystem.GrabberState state) {
        this.grabberSubsystem = grabberSubsystem;
        this.state = state;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.setGrabberState(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
