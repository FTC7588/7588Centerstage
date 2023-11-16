package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Config
public class BlueBD extends BaseOpMode {

    public static Pose2d START_POSE = new Pose2d(50, 36, Math.toRadians(0));

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    public static TrajectorySequence moveToSpike1;
    public static TrajectorySequence moveToSpike2;
    public static TrajectorySequence moveToSpike3;
    public static TrajectorySequence moveToBackdrop1;
    public static TrajectorySequence moveToBackdrop2;
    public static TrajectorySequence moveToBackdrop3;
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
        moveToSpike1 = rrDrive.trajectorySequenceBuilder(START_POSE)
                //.splineTo(new Vector2d())
                .build();
    }
}
