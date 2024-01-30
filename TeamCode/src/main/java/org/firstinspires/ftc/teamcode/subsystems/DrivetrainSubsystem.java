package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.poofyutils.MecanumDrive;
import org.firstinspires.ftc.teamcode.poofyutils.localizers.butgood.DeadwheelLocalizer;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.enums.DriveMode;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.hardware.CameraConfig;
import org.firstinspires.ftc.teamcode.poofyutils.localizers.butgood.AprilTagLocalizer2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.teamcode.Constants.*;

import android.util.Size;

import java.util.ArrayList;

public class DrivetrainSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private final MecanumDrive drive;

    private DriveMode mode;

    private double heading;

    private double fLCurrent;
    private double fRCurrent;
    private double rLCurrent;
    private double rRCurrent;

    private Pose2d dwPose;
    private Pose2d tagPose;

    private final InterpLUT blueBackdropLUT;
    private final InterpLUT redBackdropLUT;

    private final DeadwheelLocalizer dwLocalizer;
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
                THETA_COEFFS
        );

        mode = DriveMode.NONE;

        dwLocalizer = new DeadwheelLocalizer(robot);

        tagLocalizer = new AprilTagLocalizer2d(
                new CameraConfig(
                        robot.C920,
                        C920_POSE,
                        C920_INTRINSICS,
                        C920_EXPOSURE,
                        255,
                        new Size(1280, 720),
                        VisionPortal.StreamFormat.MJPEG
                )
        );

        tagPose = new Pose2d(0, 0, 0);

        blueBackdropLUT = new InterpLUT();
        blueBackdropLUT.add(-1, BLUE_BACKDROP_LEFT);
        blueBackdropLUT.add(1, BLUE_BACKDROP_RIGHT);
        blueBackdropLUT.createLUT();

        redBackdropLUT = new InterpLUT();
        redBackdropLUT.add(-1, RED_BACKDROP_LEFT);
        redBackdropLUT.add(1, RED_BACKDROP_RIGHT);
        redBackdropLUT.createLUT();

    }

    public void read() {
        if (DEBUG_DRIVE) {
            fLCurrent = robot.fL.getCurrent(AMPS);
            fRCurrent = robot.fR.getCurrent(AMPS);
            rLCurrent = robot.rL.getCurrent(AMPS);
            rRCurrent = robot.rR.getCurrent(AMPS);
        }

    }

    public void loop() {
        heading = robot.getHeading();
        dwLocalizer.update();
        dwPose = dwLocalizer.getPoseEstimate();
        tagLocalizer.update();
        tagPose = tagLocalizer.getPoseEstimate();
    }

    public void write() {
        drive.write();
    }

    public Pose2d getDwPose() {
        return dwPose;
    }

    public Pose2d getTagPose() {
        return tagPose;
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

    public void robotCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean pidTurning) {
        mode = DriveMode.ROBOT_CENTRIC;
        drive.driveRobotCentricLock(
                strafeSpeed,
                forwardSpeed,
                turnSpeed,
                robot.getHeading()
        );
        mode = DriveMode.ROBOT_CENTRIC;
    }

    public void fieldCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        mode = DriveMode.FIELD_CENTRIC;
        drive.driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                turnSpeed,
                robot.getHeading()
        );
        mode = DriveMode.FIELD_CENTRIC;
    }

    public void followTagMode(Pose2d followPose) {
        if (isDetected()) {
            drive.driveFollowTag(
                    tagPose,
                    followPose
            );
        } else {
            drive.driveRobotCentric(0, 0, 0);
        }

        mode = DriveMode.FOLLOW_TAG;
    }

    public void followBackdropMode(double range, Alliance alliance) {
        Pose2d followPose = new Pose2d(0, 0, 0);
        if (alliance == Alliance.RED) {
            followPose = new Pose2d(RED_BACKDROP.getX(), redBackdropLUT.get(range), RED_BACKDROP.getTheta());
        } else if (alliance == Alliance.BLUE) {
            followPose = new Pose2d(BLUE_BACKDROP.getX(), blueBackdropLUT.get(range), BLUE_BACKDROP.getTheta());
        }

        followTagMode(followPose);
    }

    public void followPoseMode(Pose2d targetPose) {
        drive.driveFollowPose(targetPose, dwPose, robot.getHeading());
    }

    public boolean stopped() {
        return dwLocalizer.getVelocity().getMagnitude() <= Constants.VELOCITY_THRESHOLD;
    }

    public boolean reachedTarget(double tolerance) {
        return drive.reachedTarget(tolerance, dwPose);
    }

    public boolean reachedHeading(double toleranceDeg) {
        return drive.reachedHeading(Math.toRadians(toleranceDeg), dwPose);
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

    public void setDronePosition(double pos) {
        robot.drone.setPosition(pos);
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

    public double getfLCurrent() {
        return fLCurrent;
    }

    public double getfRCurrent() {
        return fRCurrent;
    }

    public double getrLCurrent() {
        return rLCurrent;
    }

    public double getrRCurrent() {
        return rRCurrent;
    }
}
