package org.firstinspires.ftc.teamcode.opmodes.auto.t1;

import static org.firstinspires.ftc.teamcode.Constants.ARM_PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.ARM_SHOULDER_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Constants.ARM_WRIST_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Constants.FLOOR_ELE;
import static org.firstinspires.ftc.teamcode.Constants.FLOOR_SHOULDER;
import static org.firstinspires.ftc.teamcode.Constants.FLOOR_WRIST;
import static org.firstinspires.ftc.teamcode.Constants.GRABBER_CLOSED;
import static org.firstinspires.ftc.teamcode.Constants.GRABBER_OPEN;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_SHOULDER;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_WRIST;
import static org.firstinspires.ftc.teamcode.Constants.INT_DOWN;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetEleArmPositions;
import org.firstinspires.ftc.teamcode.commands.grabber.SetGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.grabber.SetLeftGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.grabber.SetRightGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Autonomous
@Disabled
@Config
public class BlueStacks extends BaseOpMode {

    public boolean runOnce = false;

    public static Pose2d START_POSE = new Pose2d(60, 36, Math.toRadians(0));

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    public static Pose2d spike1 =       new Pose2d(37, 36, Math.toRadians(-90));
    public static Pose2d spike2 =       new Pose2d(36, 39, Math.toRadians(0));
    public static Pose2d spike3 =       new Pose2d(36, 36, Math.toRadians(-90));

    public static Pose2d side =         new Pose2d(12, 36, Math.toRadians(90));
    public static Pose2d forward =      new Pose2d(12, -36, Math.toRadians(90));

    public static Pose2d backdrop1 =    new Pose2d(44, -52, Math.toRadians(90));
    public static Pose2d backdrop2 =    new Pose2d(37, -52, Math.toRadians(90));
    public static Pose2d backdrop3 =    new Pose2d(32, -52, Math.toRadians(90));
    public static Pose2d push1 =        new Pose2d(44, -56, Math.toRadians(90));
    public static Pose2d push2 =        new Pose2d(37, -56, Math.toRadians(90));
    public static Pose2d push3 =        new Pose2d(32, -56, Math.toRadians(90));

    public static Pose2d parkAll =      new Pose2d(12, -46, Math.toRadians(90));

    public static TrajectorySequence moveToSpike;
    public static TrajectorySequence moveToSide;
    public static TrajectorySequence moveAcrossField;
    public static TrajectorySequence moveToBackdrop;
    public static TrajectorySequence pushAgainstBackdrop;
    public static TrajectorySequence park;
    public static TrajectorySequence back;

    public SetEleArmPositions armBack;
    public SetEleArmPositions armDeposit;
    public SetEleArmPositions armIn;
    public SetEleArmPositions armIdle;

    public SetGrabberPosition closeBoth;
    public SetLeftGrabberPosition openLeft;
    public SetRightGrabberPosition openRight;

    public int proppos;

    public boolean pastX = false;

    @Override
    public void initialize() {
        auto = true;
        super.initialize();

        propProcessor = new PropProcessor(Alliance.BLUE);

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.C920)
                .addProcessor(propProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        autoDriveSS.setPoseEstimate(START_POSE);

        SetIntakeAngle intake = new SetIntakeAngle(intakeSS, INT_DOWN);

        SetArmPositions arm = new SetArmPositions(
                armSS, GRAB_SHOULDER, GRAB_WRIST, ARM_PIVOT_DOWN
        );

        armBack = new SetEleArmPositions(
                eleSS,
                armSS,
                FLOOR_ELE,
                FLOOR_SHOULDER,
                FLOOR_WRIST,
                ARM_PIVOT_DOWN
        );

        armDeposit = new SetEleArmPositions(
                eleSS,
                armSS,
                125,
                ARM_SHOULDER_DEPOSIT,
                ARM_WRIST_DEPOSIT,
                ARM_PIVOT_DOWN
        );

        armIn = new SetEleArmPositions(
                eleSS,
                armSS,
                0,
                GRAB_SHOULDER,
                GRAB_WRIST,
                ARM_PIVOT_DOWN
        );

        armIdle = new SetEleArmPositions(
                eleSS,
                armSS,
                200,
                GRAB_SHOULDER,
                GRAB_WRIST,
                ARM_PIVOT_DOWN
        );

        closeBoth = new SetGrabberPosition(grabSS, GRABBER_CLOSED);

        openLeft = new SetLeftGrabberPosition(grabSS, GRABBER_OPEN);

        openRight = new SetRightGrabberPosition(grabSS, GRABBER_OPEN);

        while (!isStarted() && !isStopRequested()) {
            arm.schedule();
            closeBoth.schedule();
            intake.schedule();
            robot.read(intakeSS, eleSS, armSS, grabSS);

            robot.loop(intakeSS, eleSS, armSS, grabSS);

            robot.write(intakeSS, eleSS, armSS, grabSS);

            proppos = propProcessor.getSpike();

            if (gamepad1.x && !pastX) {
                visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", 1));
            }
            pastX = gamepad1.x;

            telemetry.addData("spike pos", proppos);
            telemetry.update();
        }

    }

    @Override
    public void run() {
        super.run();

        if (!runOnce) {
            if (proppos == 1) {
                moveToSpike = autoDriveSS.trajectorySequenceBuilder(START_POSE)
                        .lineToLinearHeading(spike1)
                        .build();

                moveToSide = autoDriveSS.trajectorySequenceBuilder(moveToSpike.end())
                        .lineToLinearHeading(side)
                        .build();

                moveAcrossField = autoDriveSS.trajectorySequenceBuilder(moveToSide.end())
                        .lineToLinearHeading(forward)
                        .build();

                moveToBackdrop = autoDriveSS.trajectorySequenceBuilder(moveAcrossField.end())
                        .lineToLinearHeading(backdrop1)
                        .build();

                pushAgainstBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToBackdrop.end())
                        .lineToLinearHeading(push1)
                        .build();

                back = autoDriveSS.trajectorySequenceBuilder(pushAgainstBackdrop.end())
                        .lineToLinearHeading(backdrop1)
                        .build();
            } else if (proppos == 2) {
                moveToSpike = autoDriveSS.trajectorySequenceBuilder(START_POSE)
                        .lineToLinearHeading(spike2)
                        .build();

                moveToSide = autoDriveSS.trajectorySequenceBuilder(moveToSpike.end())
                        .lineToLinearHeading(side)
                        .build();

                moveAcrossField = autoDriveSS.trajectorySequenceBuilder(moveToSide.end())
                        .lineToLinearHeading(forward)
                        .build();

                moveToBackdrop = autoDriveSS.trajectorySequenceBuilder(moveAcrossField.end())
                        .lineToLinearHeading(backdrop2)
                        .build();

                pushAgainstBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToBackdrop.end())
                        .lineToLinearHeading(push2)
                        .build();

                back = autoDriveSS.trajectorySequenceBuilder(pushAgainstBackdrop.end())
                        .lineToLinearHeading(backdrop2)
                        .build();
            } else {
                moveToSpike = autoDriveSS.trajectorySequenceBuilder(START_POSE)
                        .lineToLinearHeading(spike3)
                        .build();

                moveToSide = autoDriveSS.trajectorySequenceBuilder(moveToSpike.end())
                        .lineToLinearHeading(side)
                        .build();

                moveAcrossField = autoDriveSS.trajectorySequenceBuilder(moveToSide.end())
                        .lineToLinearHeading(forward)
                        .build();

                moveToBackdrop = autoDriveSS.trajectorySequenceBuilder(moveAcrossField.end())
                        .lineToLinearHeading(backdrop3)
                        .build();

                pushAgainstBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToBackdrop.end())
                        .lineToLinearHeading(push3)
                        .build();

                back = autoDriveSS.trajectorySequenceBuilder(pushAgainstBackdrop.end())
                        .lineToLinearHeading(backdrop3)
                        .build();
            }



            park = autoDriveSS.trajectorySequenceBuilder(back.end())
                    .lineToLinearHeading(parkAll)
                    .build();

            pathSchedule();

            runOnce = true;
        }

        robot.read(intakeSS, eleSS, armSS, grabSS);

        robot.loop(intakeSS, eleSS, armSS, grabSS);

        robot.write(intakeSS, eleSS, armSS, grabSS);

        telemetry.addData("Spike Position", propProcessor.getSpike());
        telemetry.update();

        robot.clearBulkCache();
    }

    public void pathSchedule() {
        schedule(
                new SequentialCommandGroup(
                        //move to spike
                        new FollowTrajectorySequenceAsync(autoDriveSS, moveToSpike),
                        armBack,
                        new WaitCommand(800),

                        //drop pixel
                        openLeft,
                        new WaitCommand(800),

                        //move to side
                        armIdle,
                        new FollowTrajectorySequenceAsync(autoDriveSS, moveToSide),
                        new WaitCommand(400),

                        //cross field
                        new FollowTrajectorySequenceAsync(autoDriveSS, moveAcrossField),
                        new WaitCommand(400),

                        //go to backdrop
                        new FollowTrajectorySequenceAsync(autoDriveSS, moveToBackdrop),
                        armDeposit,
                        new WaitCommand(800),

                        //push against backdrop
                        new FollowTrajectorySequenceAsync(autoDriveSS, pushAgainstBackdrop),
                        openRight,
                        new WaitCommand(200),
                        new FollowTrajectorySequenceAsync(autoDriveSS, back),
                        new WaitCommand(400),

                        //park
                        armIn,
                        new FollowTrajectorySequenceAsync(autoDriveSS, park),
                        new WaitCommand(800)
                )
        );
    }
}
