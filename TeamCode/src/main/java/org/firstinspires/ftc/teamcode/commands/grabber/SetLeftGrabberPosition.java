package org.firstinspires.ftc.teamcode.commands.grabber;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GrabberSubsystem;

public class SetLeftGrabberPosition extends CommandBase {

    private final GrabberSubsystem m_grabberSubsystem;

    private final double pos;

    public SetLeftGrabberPosition(GrabberSubsystem grabberSubsystem, double pos) {
        m_grabberSubsystem = grabberSubsystem;
        addRequirements(m_grabberSubsystem);
        this.pos = pos;
    }

    @Override
    public void initialize() {
        m_grabberSubsystem.setLeftGrabberPosition(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
