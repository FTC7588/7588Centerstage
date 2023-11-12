package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.poofyutils.MecanumDrive;
import org.firstinspires.ftc.teamcode.poofyutils.enums.DriveMode;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.hardware.CameraConfig;
import org.firstinspires.ftc.teamcode.poofyutils.localizers.butgood.AprilTagLocalizer2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import static org.firstinspires.ftc.teamcode.Constants.*;

import android.util.Size;

import java.util.ArrayList;

public class DrivetrainSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private final MecanumDrive drive;

    private DriveMode mode;

    private double heading;

    private Pose2d robotPose;

    private final AprilTagLocalizer2d tagLocalizer;

    public DrivetrainSubsystem(RobotHardware robot) {
        this.robot = robot;

        drive = new MecanumDrive(
                robot.fL,
                robot.fR,
                robot.rL,
                robot.rR,
                X_COEFFS,
                Y_COEFFS,
                THETA_COEFFS,
                0
        );

        mode = DriveMode.NONE;

        tagLocalizer = new AprilTagLocalizer2d(
                new CameraConfig(
                        robot.C920,
                        C920_POSE,
                        C920_INTRINSICS,
                        12,
                        255,
                        new Size(640, 480),
                        VisionPortal.StreamFormat.MJPEG
                )
        );

        robotPose = new Pose2d(0, 0, 0);

    }

    public void read() {

    }

    public void loop() {
        heading = robot.getHeading();
        tagLocalizer.update();
        robotPose = tagLocalizer.getPoseEstimate();
    }

    public void write() {
        drive.write();
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public ArrayList<AprilTagDetection> getUsedTags() {
        return tagLocalizer.getUsedTags();
    }

    public AprilTagLocalizer2d getTagLocalizer() {
        return tagLocalizer;
    }

    public boolean isDetected() {
        return tagLocalizer.isDetected();
    }
//
//    public Pose2d getTagPose() {
//        return tagLocalizer.getTagPose();
//    }
//
//    public Pose2d getCamPose() {
//        return tagLocalizer.getCamPose();
//    }
//
//    public Transform2d getCamToTag() {
//        return tagLocalizer.getCamToTag();
//    }

    public void robotCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean pidTurning) {
        if (!pidTurning) {
            mode = DriveMode.ROBOT_CENTRIC;
            drive.driveRobotCentricLock(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    getHeading()
            );
        } else {
            mode = DriveMode.ROBOT_CENTRIC_PID;
            drive.driveRobotCentricPID(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    heading
            );
        }
        mode = DriveMode.ROBOT_CENTRIC;
    }

    public void fieldCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean pidTurning) {
        if (!pidTurning) {
            mode = DriveMode.FIELD_CENTRIC;
            drive.driveFieldCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    robot.getHeading()
            );
        } else {
            mode = DriveMode.FIELD_CENTRIC_PID;
            drive.driveFieldCentricPID(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    heading
            );
        }
        mode = DriveMode.FIELD_CENTRIC;
    }

    public void followTagMode(Pose2d followPose) {
        if (isDetected()) {
            drive.driveFollowTag(
                    robotPose,
                    followPose
            );
        } else {
            drive.driveRobotCentric(0, 0, 0);
        }

        mode = DriveMode.FOLLOW_TAG;
    }

    //setters
    public void setMaxSpeed(double speed) {
        drive.setMaxSpeed(speed);
    }

    public void setHeadingLock(boolean enabled) {
        drive.setHeadingLock(enabled);
    }

    public void setHeadingLockTarget(double target) {
        drive.setHeadingLockTarget(target);
    }

    //getters
    public double getHeading() {
        return Math.toDegrees(heading);
    }

    public DriveMode getMode() {
        return mode;
    }

    public double getMaxSpeed() {
        return drive.getMaxSpeed();
    }

    public boolean getHeadingLock() {
        return drive.getHeadingLock();
    }

    public double getHeadingLockTarget() {
        return drive.getHeadingLockTarget();
    }

}
