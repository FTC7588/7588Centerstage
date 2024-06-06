package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class HoldTargetPosition extends CommandBase {

    private final DrivetrainSubsystem drivetrainSubsystem;

    public HoldTargetPosition(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(this.drivetrainSubsystem);
    }

    @Override
    public void execute() {
        drivetrainSubsystem.followPoseMode(drivetrainSubsystem.getTargetPose(), 0.01, 0.01);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.robotCentricMode(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
//        return (drivetrainSubsystem.reachedTarget(positionTolerance) && drivetrainSubsystem.reachedHeading(headingTolerance));
    }
}
