package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectoryAsyncCommand;
import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.arm.SetEleArmPositions;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import static org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous
@Config
public class BlueBD extends BaseOpMode {

    public static Pose2d START_POSE = new Pose2d(50, 36, Math.toRadians(0));

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    public static Pose2d spike1;
    public static Pose2d spike2;
    public static Pose2d spike3;
    public static Pose2d backdrop1;
    public static Pose2d backdrop2;
    public static Pose2d backdrop3;

    public static Pose2d parkall;

    public static TrajectorySequence moveToSpike;
    public static TrajectorySequence moveToBackdrop;
    public static TrajectorySequence park;

    @Override
    public void initialize() {
        auto = true;
        super.initialize();

        propProcessor = new PropProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.C920)
                .addProcessor(propProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();
    }

    @Override
    public void run() {
        super.run();

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
                        new ParallelCommandGroup(
                                new FollowTrajectorySequenceAsync(autoDriveSS, moveToSpike),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new SetEleArmPositions(
                                            eleSS,
                                            armSS,
                                            FLOOR_ELE,
                                            FLOOR_SHOULDER,
                                            FLOOR_WRIST,
                                            ARM_PIVOT_DOWN
                                        )
                                )
                        ),
                        //drop pixel
                        new ParallelCommandGroup(),
                        //go to backdrop
                        new ParallelCommandGroup(),
                        //push against backdrop
                        new ParallelCommandGroup(),
                        //park
                        new ParallelCommandGroup()
                )
        );
    }
}
