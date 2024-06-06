package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class SetDronePosition extends CommandBase {

    private final double pos;
    private final DrivetrainSubsystem drivetrainSubsystem;

    public SetDronePosition(DrivetrainSubsystem drivetrainSubsystem, double pos) {
        this.pos = pos;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void initialize() {
        super.initialize();
        drivetrainSubsystem.setDronePosition(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
