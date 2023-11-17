package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectoryAsyncCommand;
import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetEleArmPositions;
import org.firstinspires.ftc.teamcode.commands.grabber.SetGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.grabber.SetLeftGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.grabber.SetRightGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import static org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous
@Config
public class BlueBD extends BaseOpMode {

    public boolean runOnce = false;



    public static Pose2d START_POSE = new Pose2d(60, -12, Math.toRadians(0));

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    public static Pose2d spike1 =       new Pose2d(32, -34, Math.toRadians(-90));
    public static Pose2d spike2 =       new Pose2d(36, -12, Math.toRadians(0));
    public static Pose2d spike3 =       new Pose2d(36, -12, Math.toRadians(-90));
    public static Pose2d backdrop1 =    new Pose2d(40, -52, Math.toRadians(90));
    public static Pose2d backdrop2 =    new Pose2d(35, -52, Math.toRadians(90));
    public static Pose2d backdrop3 =    new Pose2d(32, -52, Math.toRadians(90));
    public static Pose2d push1 =        new Pose2d(40, -56, Math.toRadians(90));
    public static Pose2d push2 =        new Pose2d(35, -56, Math.toRadians(90));
    public static Pose2d push3 =        new Pose2d(32, -56, Math.toRadians(90));

    public static Pose2d parkall =      new Pose2d(54, -46, Math.toRadians(90));

    public static TrajectorySequence moveToSpike;
    public static TrajectorySequence moveToBackdrop;
    public static TrajectorySequence pushAgainstBackdrop;
    public static TrajectorySequence park;
    public static TrajectorySequence back;

    public SetEleArmPositions armBack;
    public SetEleArmPositions armDeposit;
    public SetEleArmPositions armIn;

    public SetGrabberPosition closeBoth;
    public SetLeftGrabberPosition openLeft;
    public SetRightGrabberPosition openRight;

    @Override
    public void initialize() {
        auto = true;
        super.initialize();

        propProcessor = new PropProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.C920)
                .setLiveViewContainerId(0)
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
                200,
                ARM_SHOULDER_DEPOSIT,
                ARM_WRIST_DEPOSIT,
                ARM_PIVOT_DOWN
        );

        armIn = new SetEleArmPositions(
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

            telemetry.addData("spike pos", propProcessor.getSpike());
            telemetry.update();
        }


    }

    @Override
    public void run() {
        super.run();

        if (!runOnce) {
            if (propProcessor.getSpike() == 1) {
                moveToSpike = autoDriveSS.trajectorySequenceBuilder(START_POSE)
                        .lineToLinearHeading(spike1)
                        .build();

                moveToBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToSpike.end())
                        .lineToLinearHeading(backdrop1)
                        .build();

                pushAgainstBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToBackdrop.end())
                        .lineToLinearHeading(push1)
                        .build();
            } else if (propProcessor.getSpike() == 2) {
                moveToSpike = autoDriveSS.trajectorySequenceBuilder(START_POSE)
                        .lineToLinearHeading(spike2)
                        .build();

                moveToBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToSpike.end())
                        .lineToLinearHeading(backdrop2)
                        .build();

                pushAgainstBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToBackdrop.end())
                        .lineToLinearHeading(push2)
                        .build();
            } else {
                moveToSpike = autoDriveSS.trajectorySequenceBuilder(START_POSE)
                        .lineToLinearHeading(spike3)
//                        .addTemporalMarker(0.5, armBack::schedule)
                        .build();

                moveToBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToSpike.end())
                        .lineToLinearHeading(backdrop3)
//                        .addTemporalMarker(0.5, armDeposit::schedule)
                        .build();

                pushAgainstBackdrop = autoDriveSS.trajectorySequenceBuilder(moveToBackdrop.end())
                        .lineToLinearHeading(push3)
                        .build();

                back = autoDriveSS.trajectorySequenceBuilder(pushAgainstBackdrop.end())
                        .lineToLinearHeading(backdrop3)
                        .build();
            }
            park = autoDriveSS.trajectorySequenceBuilder(back.end())
                    .lineToLinearHeading(parkall)
//                    .addTemporalMarker(0.1, armIn::schedule)
                    .build();

            pathSchedule();

            runOnce = true;
        }

        robot.read(intakeSS, eleSS, armSS, grabSS);

        robot.loop(intakeSS, eleSS, armSS, grabSS);

        robot.write(intakeSS, eleSS, armSS, grabSS);

        telemetry.addData("Spike Position", propProcessor.getSpike());
        telemetry.update();
    }

    public void instantiateTrajectories() {
        moveToSpike = rrDrive.trajectorySequenceBuilder(START_POSE)
                //.splineTo(new Vector2d())
                .build();
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

                        //go to backdrop
                        armIn,
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
