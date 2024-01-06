package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.Blue.*;

import android.util.Size;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.enums.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous
public class BlueW extends BaseOpMode {

    public static Paths path = Paths.W_1_C;
    private int list = 2;

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    private final double WAIT_BD = 1;
    private final double WAIT_SPIKE = 1;

    private TrajectorySequence toSpike;

    private TrajectorySequence toAStackFromOne;
    private TrajectorySequence toAStackFromTwo;
    private TrajectorySequence toAStackFromThree;

    private TrajectorySequence toBStackFromOne;
    private TrajectorySequence toBStackFromTwo;
    private TrajectorySequence toBStackFromThree;

    private TrajectorySequence toCStackFromOne;
    private TrajectorySequence toCStackFromTwo;
    private TrajectorySequence toCStackFromThree;

    private TrajectorySequence toCStackFromBD;

    private TrajectorySequence toBD2FromCStack;
    private TrajectorySequence toStackFromTwo;
    private TrajectorySequence toStackFromThree;
    private TrajectorySequence toBDFromCStack;
    private TrajectorySequence toBDFromSpike;
    private TrajectorySequence push;
    private TrajectorySequence park;

    private int propPos;

    private boolean pastX = false;

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

        autoDriveSS.setPoseEstimate(W_START);
    }

    @Override
    public void initLoop() {
        new SetIntakeAngle(intakeSS, INT_UP);
        armGrab.schedule();
        grabberLeftClose.schedule();
        grabberRightOpen.schedule();
//        grabbersClosed.schedule();

        robot.read(intakeSS, eleSS, armSS, grabSS);

        robot.loop(intakeSS, eleSS, armSS, grabSS);

        robot.write(intakeSS, eleSS, armSS, grabSS);

        propPos = propProcessor.getSpike();

        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", 1));
        }

        if (gamepad1.a && !pastX) {
            if (list < 4) {
                list++;
            } else {
                list = 0;
            }
        }
        gamepad1.a = pastX;

        switch (list) {
            case 0:
                path = Paths.W_0;
                break;
            case 1:
                path = Paths.W_1_C;
                break;
            case 2:
                path = Paths.W_3_C;
                break;
            case 3:
                path = Paths.W_5_C;
                break;
        }

        tad("spike pos", propPos);
        tad("path", path);
        tau();
    }

    @Override
    public void runOnce() {

        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                .lineToLinearHeading(W_SPIKE_ONE)
                .build();

        toCStackFromOne = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                .lineToLinearHeading(STACK_C)
                .build();

        toBDFromCStack = autoDriveSS.trajectorySequenceBuilder(toCStackFromOne.end())
                .setReversed(true)
                .splineToSplineHeading(STACK_BD_1_A, STACK_BD_1_A_TANGENT)
                .splineToLinearHeading(BD_ONE_OFF, Math.toRadians(0))
                .build();

        toCStackFromBD = autoDriveSS.trajectorySequenceBuilder(toBDFromCStack.end())
                .setReversed(false)
                .splineToLinearHeading(STACK_BD_1_A,Math.toRadians(180))
                .splineToSplineHeading(STACK_C,Math.toRadians(180))
                .build();

        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                .lineToLinearHeading(W_BD_ONE_A)
                .splineToLinearHeading(W_BD_ONE_B, W_BD_ONE_B_TANGENT)
                .splineToLinearHeading(BD_ONE_OFF, W_BD_ONE_C_TANGENT)
                .build();

        toBD2FromCStack = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD.end())
                .setReversed(true)
                .splineToSplineHeading(STACK_BD_1_A, STACK_BD_1_A_TANGENT)
                .splineToLinearHeading(BD_TWO_OFF, Math.toRadians(0))
                .build();

        push = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                .lineToLinearHeading(BD_ONE_PUSH)
                .build();

        park = autoDriveSS.trajectorySequenceBuilder(push.end())
                .lineToLinearHeading(PARK_CENTER)
                .build();

        switch (path) {
            case W_0:
                tal("W_0");
                schedule(
                        new SequentialCommandGroup(
                                new FollowTrajectorySequenceAsync(autoDriveSS, toSpike),
                                armBackGroup,
                                new WaitCommand(500),
                                pivotPosDown,
                                new WaitCommand(100),
                                grabberLeftOpen,
                                new WaitCommand(100),
                                armGrab,
                                new WaitCommand(250),
                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromSpike),
                                armDepositGroup,
                                new WaitCommand(500),
                                new FollowTrajectorySequenceAsync(autoDriveSS, push),
                                grabberRightOpen,
                                new WaitCommand(500),
                                armGrab,
                                new FollowTrajectorySequenceAsync(autoDriveSS, park)
                        )
                );
                break;
            case W_1_C:
                tal("W_1_C");
                schedule(
                        new SequentialCommandGroup(
                                new FollowTrajectorySequenceAsync(autoDriveSS, toSpike),
                                armBackGroup,
                                new WaitCommand(500),
                                grabberLeftOpen,
                                new WaitCommand(200),
                                armGrab,
                                new WaitCommand(250),
                                new FollowTrajectorySequenceAsync(autoDriveSS, toCStackFromOne),

                                new WaitCommand(100),
                                new SetIntakePower(intakeSS, -1),
                                new WaitCommand(250),
                                new SetIntakeAngle(intakeSS, INT_FIVE),
                                new WaitCommand(2000),
                                intakeIdle,
                                new WaitCommand(500),
                                grabbersClosed,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStack),
                                armDepositGroup,
                                new WaitCommand(500),
                                new FollowTrajectorySequenceAsync(autoDriveSS, push),
                                grabbersOpen,
                                new WaitCommand(500),
                                armGrab,
                                new FollowTrajectorySequenceAsync(autoDriveSS, park)
                        )
                );
                break;

            case W_3_C:
                tal("W_3_C");
                schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new FollowTrajectorySequenceAsync(autoDriveSS, toSpike),
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                armBackGroup
                                        )
                                ),
                                new WaitCommand(400),
                                grabberLeftOpen,
                                new WaitCommand(200),
                                armGrab,
                                new WaitCommand(250),
                                new FollowTrajectorySequenceAsync(autoDriveSS, toCStackFromOne),

                                new WaitCommand(100),
                                new SetIntakePower(intakeSS, -1),
                                new WaitCommand(250),
                                new SetIntakeAngle(intakeSS, INT_FIVE),
                                new WaitCommand(1000),
                                intakeIdle,
                                new WaitCommand(500),
                                grabbersClosed,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStack),
                                armDepositGroup,
                                new WaitCommand(400),
                                new FollowTrajectorySequenceAsync(autoDriveSS, push),
                                grabbersOpen,
                                new WaitCommand(400),
                                armGrab,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toCStackFromBD),
                                new WaitCommand(100),
                                new SetIntakePower(intakeSS, -1),
                                new WaitCommand(250),
                                new SetIntakeAngle(intakeSS, INT_FOUR),
                                new WaitCommand(1000),
                                new SetIntakeAngle(intakeSS, INT_THREE),
                                new WaitCommand(1000),
                                intakeIdle,
                                new WaitCommand(500),
                                grabbersClosed,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBD2FromCStack),
                                armDepositGroup,
                                new WaitCommand(500),
                                new FollowTrajectorySequenceAsync(autoDriveSS, push),
                                grabbersOpen,
                                new WaitCommand(500),
                                armGrab,

                                new FollowTrajectorySequenceAsync(autoDriveSS, park)
                        )
                );
                break;
        }
        tau();
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
