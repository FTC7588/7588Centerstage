package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class SetMaxSpeed extends CommandBase {

    public DrivetrainSubsystem m_drivetrainSubsystem;

    public double speed;

    public SetMaxSpeed(DrivetrainSubsystem drivetrainSubsystem, double speed) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.setMaxSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
