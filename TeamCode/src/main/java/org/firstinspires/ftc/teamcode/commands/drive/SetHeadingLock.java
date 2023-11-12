package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class SetHeadingLock extends CommandBase {

    public DrivetrainSubsystem m_drivetrainSubsystem;

    public double target;

    public SetHeadingLock(DrivetrainSubsystem drivetrainSubsystem, double target) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        this.target = target;
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.setHeadingLockTarget(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
