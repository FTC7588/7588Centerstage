package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class PIDToPoint extends CommandBase {

    private final Pose2d targetPose;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final double positionTolerance, headingTolerance;

    public PIDToPoint(DrivetrainSubsystem drivetrainSubsystem, Pose2d targetPose, double positionTolerance, double headingTolerance) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(this.drivetrainSubsystem);
        this.targetPose = targetPose;
        this.positionTolerance = positionTolerance;
        this.headingTolerance = headingTolerance;
    }

    @Override
    public void execute() {
        drivetrainSubsystem.followPoseMode(targetPose);
    }

    @Override
    public boolean isFinished() {
        return (drivetrainSubsystem.reachedTarget(positionTolerance) && drivetrainSubsystem.reachedHeading(headingTolerance));
    }
}
