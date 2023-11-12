package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class EnableHeadingLock extends CommandBase {

    private DrivetrainSubsystem m_drivetrainSubsystem;

    private boolean enabled;

     public EnableHeadingLock(DrivetrainSubsystem drivetrainSubsystem, boolean enabled) {
         m_drivetrainSubsystem = drivetrainSubsystem;
         this.enabled = enabled;
     }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.setHeadingLock(enabled);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
