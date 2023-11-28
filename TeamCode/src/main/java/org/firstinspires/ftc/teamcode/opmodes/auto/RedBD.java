package org.firstinspires.ftc.teamcode.opmodes.auto;

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

import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetEleArmPositions;
import org.firstinspires.ftc.teamcode.commands.grabber.SetGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.grabber.SetLeftGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.grabber.SetRightGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.enums.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Autonomous
@Config
public class RedBD extends BaseOpMode {

    public boolean runOnce = false;

    public static Pose2d START_POSE = new Pose2d(-60, -12, Math.toRadians(180));

    private PropProcessor propProcessor;

    public static Pose2d spike3 =       new Pose2d(-33, -32.5, Math.toRadians(-90));
    public static Pose2d spike2 =       new Pose2d(-34, -16, Math.toRadians(180));
    public static Pose2d spike1 =       new Pose2d(-35, -9, Math.toRadians(-90));
    public static Pose2d backdrop3 =    new Pose2d(-42, -50, Math.toRadians(90));
    public static Pose2d backdrop2 =    new Pose2d(-36, -50, Math.toRadians(90));
    public static Pose2d backdrop1 =    new Pose2d(-28, -50, Math.toRadians(90));
    public static Pose2d push3 =        new Pose2d(-42, -53, Math.toRadians(90));
    public static Pose2d push2 =        new Pose2d(-35, -53, Math.toRadians(90));
    public static Pose2d push1 =        new Pose2d(-30, -53, Math.toRadians(90));

    public static Pose2d parkAll =      new Pose2d(-60, -46, Math.toRadians(90));

    public static TrajectorySequence moveToSpike;
    public static TrajectorySequence moveToBackdrop;
    public static TrajectorySequence pushAgainstBackdrop;
    public static TrajectorySequence park;
    public static TrajectorySequence back;

    public int proppos;

    public boolean pastX;

    @Override
    public void initialize() {
        auto = true;
        super.initialize();

        propProcessor = new PropProcessor(Alliance.RED);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(robot.C920)
                .addProcessor(propProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        autoDriveSS.setPoseEstimate(START_POSE);

        while (!isStarted() && !isStopRequested()) {
            armInit.schedule();
            grabbersClosed.schedule();
            intakeDown.schedule();
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

                moveToBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToSpike.end())
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

                moveToBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToSpike.end())
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

                moveToBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToSpike.end())
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
                        grabberLeftOpen,
                        new WaitCommand(800),

                        //go to backdrop
                        armIdle,
                        new FollowTrajectorySequenceAsync(autoDriveSS, moveToBackdrop),
                        armDeposit,
                        new WaitCommand(800),

                        //push against backdrop
                        new FollowTrajectorySequenceAsync(autoDriveSS, pushAgainstBackdrop),
                        grabberRightOpen,
                        new WaitCommand(200),
                        new FollowTrajectorySequenceAsync(autoDriveSS, back),
                        new WaitCommand(400),

                        //park
                        armDown,
                        new FollowTrajectorySequenceAsync(autoDriveSS, park),
                        new WaitCommand(800)
                )
        );
    }
}
