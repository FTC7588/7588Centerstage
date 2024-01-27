package org.firstinspires.ftc.teamcode.commands.grabber;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GrabberSubsystem;

public class SetGrabberPosition extends CommandBase {

    private final GrabberSubsystem m_grabberSubsystem;

    private final double pos1;
    private final double pos2;

    public SetGrabberPosition(GrabberSubsystem grabberSubsystem, double pos1, double pos2) {
        m_grabberSubsystem = grabberSubsystem;
        addRequirements(m_grabberSubsystem);
        this.pos1 = pos1;
        this.pos2 = pos2;
    }

    @Override
    public void initialize() {
        m_grabberSubsystem.setGrabberPositions(pos1, pos2);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
