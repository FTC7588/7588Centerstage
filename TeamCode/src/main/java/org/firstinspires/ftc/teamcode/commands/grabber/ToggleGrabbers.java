package org.firstinspires.ftc.teamcode.commands.grabber;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GrabberSubsystem;

public class ToggleGrabbers extends CommandBase {

    private final GrabberSubsystem m_grabberSubsystem;

    public ToggleGrabbers(GrabberSubsystem grabberSubsystem) {
        m_grabberSubsystem = grabberSubsystem;
        addRequirements(m_grabberSubsystem);
    }

    @Override
    public void initialize() {
        m_grabberSubsystem.toggleGrabbers();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

