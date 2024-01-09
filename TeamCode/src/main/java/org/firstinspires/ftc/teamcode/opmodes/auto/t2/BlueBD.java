package org.firstinspires.ftc.teamcode.opmodes.auto.t2;

import static org.firstinspires.ftc.teamcode.opmodes.auto.t2.AutoConstants.Blue.*;

import android.util.Size;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.enums.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Autonomous
public class BlueBD extends BaseOpMode {

    public static Paths path = Paths.BS_0;
    private int list = 0;

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    private final double WAIT_BD = 1;
    private final double WAIT_SPIKE = 1;

    private TrajectorySequence toSpike;
    private TrajectorySequence toBD;
    private TrajectorySequence push;
    private TrajectorySequence park;

    private int propPos;

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

        autoDriveSS.setPoseEstimate(BD_START);
    }

    @Override
    public void initLoop() {
        intakeUp.schedule();
        armGrab.schedule();
        grabbersClosed.schedule();

        robot.read(intakeSS, eleSS, armSS, grabSS);

        robot.loop(intakeSS, eleSS, armSS, grabSS);

        robot.write(intakeSS, eleSS, armSS, grabSS);

        propPos = propProcessor.getSpike();

        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", 1));
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            if (list < 4) {
                list++;
            } else {
                list = 0;
            }
        }

        switch (list) {
            case 0:
                path = Paths.BS_0;
                break;
            case 1:
                path = Paths.BS_2_A;
                break;
            case 2:
                path = Paths.BS_4_A;
                break;
            case 3:
                path = Paths.BS_6_A;
                break;
        }

        tad("spike pos", propPos);
        tad("path", path);
        tau();
    }

    @Override
    public void runOnce() {

        toSpike = autoDriveSS.trajectorySequenceBuilder(BD_START)
                .lineToLinearHeading(BD_SPIKE_ONE)
                .build();

        toBD = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                .lineToLinearHeading(BD_ONE_OFF)
                .build();

        push = autoDriveSS.trajectorySequenceBuilder(toBD.end())
                .lineToLinearHeading(BD_ONE_PUSH)
                .build();

        park = autoDriveSS.trajectorySequenceBuilder(push.end())
                .lineToLinearHeading(PARK_CORNER)
                .build();

        switch (path) {
            case BS_0:
                schedule(
                        new SequentialCommandGroup(
                                new FollowTrajectorySequenceAsync(autoDriveSS, toSpike),

                                new SetIntakePower(intakeSS, 0.5),
                                new WaitCommand(500),
                                intakeIdle,

                                armDepositGroup,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBD),

                                new FollowTrajectorySequenceAsync(autoDriveSS, push),

                                grabberLeftOpen,
                                new WaitCommand(500),

                                armGrab,

                                new FollowTrajectorySequenceAsync(autoDriveSS, park)
                        )
                );
                break;
            case BS_2_A:

                break;
        }
    }

    @Override
    public void run() {
        super.run();

        robot.read(intakeSS, eleSS, armSS, grabSS);

        robot.loop(intakeSS, eleSS, armSS, grabSS);

        robot.write(intakeSS, eleSS, armSS, grabSS);

        robot.clearBulkCache();
    }
}
