package org.firstinspires.ftc.teamcode.commands._rr;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AutoDrivetrainSubsystem;

public class FollowTrajectorySequenceAsync extends CommandBase {

    private final AutoDrivetrainSubsystem drive;
    private final TrajectorySequence traj;

    public FollowTrajectorySequenceAsync(AutoDrivetrainSubsystem drive, TrajectorySequence traj) {
        this.drive = drive;
        this.traj = traj;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(traj);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }

}
