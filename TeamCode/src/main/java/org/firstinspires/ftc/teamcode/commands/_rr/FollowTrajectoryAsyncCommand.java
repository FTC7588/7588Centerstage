package org.firstinspires.ftc.teamcode.commands._rr;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AutoDrivetrainSubsystem;

public class FollowTrajectoryAsyncCommand extends CommandBase {

    private final AutoDrivetrainSubsystem drive;
    private final Trajectory traj;

    public FollowTrajectoryAsyncCommand(AutoDrivetrainSubsystem drive, Trajectory traj) {
        this.drive = drive;
        this.traj = traj;
    }

    @Override
    public void initialize() {
        drive.followTrajectoryAsync(traj);
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
