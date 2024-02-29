package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class PIDToPointSpeed extends CommandBase {

    private final Pose2d targetPose;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final double positionTolerance, headingTolerance, speed;

    public PIDToPointSpeed(DrivetrainSubsystem drivetrainSubsystem, Pose2d targetPose, double positionTolerance, double headingTolerance, double speed) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(this.drivetrainSubsystem);
        this.targetPose = targetPose;
        this.positionTolerance = positionTolerance;
        this.headingTolerance = headingTolerance;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.setTargetPose(targetPose);
        drivetrainSubsystem.setPositionTolerance(positionTolerance);
        drivetrainSubsystem.setHeadingTolerance(headingTolerance);
        drivetrainSubsystem.setMaxSpeed(speed);
    }

    @Override
    public void execute() {
        drivetrainSubsystem.followPoseMode();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setMaxSpeed(1);
        drivetrainSubsystem.robotCentricMode(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
//        return false;
        return (drivetrainSubsystem.reachedTarget(positionTolerance) && drivetrainSubsystem.reachedHeading(headingTolerance));
    }
}
