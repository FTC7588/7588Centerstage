package org.firstinspires.ftc.teamcode.opmodes.auto.t2;

import static org.firstinspires.ftc.teamcode.Constants.INT_FIVE;
import static org.firstinspires.ftc.teamcode.Constants.INT_FOUR;
import static org.firstinspires.ftc.teamcode.Constants.INT_THREE;
import static org.firstinspires.ftc.teamcode.Constants.INT_UP;
import static org.firstinspires.ftc.teamcode.opmodes.auto.t2.AutoConstants.Red.*;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commands.arm.SetPivotPosition;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Autonomous
@Config
@Disabled
public class RedW extends BaseOpMode {

    public static Paths path = Paths.W_1_C;
    private int list = 2;

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    private final double WAIT_BD = 1;
    private final double WAIT_SPIKE = 1;

    private TrajectorySequence toSpike;
    private TrajectorySequence toCStackFromSpike;
    private TrajectorySequence toCStackFromBD;
    private TrajectorySequence toBDFromCStackAA;
    private TrajectorySequence toBDFromCStackAB;
    private TrajectorySequence toBDFromCStackBA;
    private TrajectorySequence toBDFromCStackBB;
    private TrajectorySequence toBDFromSpike;
    private TrajectorySequence park;

    private SetPivotPosition pivotPosition;

    public static int propPos = 3;

    private boolean pastA = false;
    private boolean pastX = false;

    @Override
    public void initialize() {
        Constants.ELE_PID = false;
        auto = true;
        super.initialize();

        propProcessor = new PropProcessor(Alliance.RED_W);

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
        new SetIntakeAngle(intakeSS, INT_UP).schedule();
        armGrab.schedule();
        grabberLeftClose.schedule();
        grabberRightOpen.schedule();
//        grabbersClosed.schedule();

        robot.read(intakeSS, eleSS, armSS, grabSS);

        robot.loop(intakeSS, eleSS, armSS, grabSS);

        robot.write(intakeSS, eleSS, armSS, grabSS);

        if (propProcessor.getSpike() != 0) {
            propPos = propProcessor.getSpike();
        }

        //take picture
        if (gamepad1.x && !pastX) {
            visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", System.nanoTime()));
            tal("Picture Taken!");
        }
        gamepad1.x = pastX;

        //change path type
        if (gamepad1.a && !pastA) {
            if (list < 4) {
                list++;
            } else {
                list = 0;
            }
        }
        gamepad1.a = pastA;

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

        switch (path) {
            case W_0:
                switch (propPos) {
                    case 1:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_ONE)
                                .build();

                        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(W_BD_ONE_A)
                                .lineToLinearHeading(W_BD_ONE_B)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        pivotPosition = pivotPosUp;
                        break;
                    case 2:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_TWO_ALT)
                                .build();

                        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(W_BD_TWO_A)
                                .lineToLinearHeading(W_BD_TWO_B)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        pivotPosition = pivotPosUp;
                        break;
                    case 3:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_THREE)
                                .build();

                        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(W_BD_THREE_A)
                                .lineToLinearHeading(W_BD_THREE_B)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        pivotPosition = pivotPosDown;
                        break;
                }
                break;
            case W_1_C:
                switch (propPos) {
                    case 1:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_ONE)
                                .build();

                        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(STACK_C_1)
                                .build();

                        toBDFromCStackAA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .build();

                        toBDFromCStackAB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAA.end())
                                .lineToLinearHeading(W_BD_ONE_THREE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAB.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        pivotPosition = pivotPosUp;
                        break;
                    case 2:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_TWO_ALT)
                                .build();

                        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(STACK_C_2)
                                .build();

                        toBDFromCStackAA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .build();

                        toBDFromCStackAB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAA.end())
                                .lineToLinearHeading(W_BD_TWO_THREE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAB.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        pivotPosition = pivotPosUp;
                        break;
                    case 3:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_THREE)
                                .build();

                        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(STACK_C_3)
                                .build();

                        toBDFromCStackAA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .build();

                        toBDFromCStackAB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAA.end())
                                .lineToLinearHeading(W_BD_THREE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAB.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        pivotPosition = pivotPosDown;
                        break;
                }
                break;
            case W_3_C:
                switch (propPos) {
                    case 1:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_ONE)
                                .build();

                        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(STACK_C_1)
                                .build();

                        toBDFromCStackAA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .build();

                        toBDFromCStackAB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAA.end())
                                .lineToLinearHeading(W_BD_ONE_ONE_OFF)
                                .build();

                        toCStackFromBD = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAB.end())
                                .setReversed(false)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(STACK_C_1)
                                .build();

                        toBDFromCStackBA = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .build();

                        toBDFromCStackBB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackBA.end())
                                .lineToLinearHeading(W_BD_TWO_ONE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackBB.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        pivotPosition = pivotPosUp;
                        break;
                    case 2:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_TWO_ALT)
                                .build();

                        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(STACK_C_2)
                                .build();

                        toBDFromCStackAA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .build();

                        toBDFromCStackAB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAA.end())
                                .lineToLinearHeading(W_BD_TWO_TWO_OFF)
                                .build();

                        toCStackFromBD = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAB.end())
                                .setReversed(false)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(STACK_C_2)
                                .build();

                        toBDFromCStackBA = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .build();

                        toBDFromCStackBB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackBA.end())
                                .lineToLinearHeading(W_BD_ONE_TWO_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackBB.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        pivotPosition = pivotPosUp;
                        break;
                    case 3:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_THREE)
                                .build();

                        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(STACK_C_3)
                                .build();

                        toBDFromCStackAA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .build();

                        toBDFromCStackAB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAA.end())
                                .lineToLinearHeading(W_BD_THREE_OFF)
                                .build();

                        toCStackFromBD = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackAB.end())
                                .setReversed(false)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(STACK_C_3)
                                .build();

                        toBDFromCStackBA = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .build();

                        toBDFromCStackBB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackBA.end())
                                .lineToLinearHeading(W_BD_TWO_THREE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackBB.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        pivotPosition = pivotPosDown;
                        break;
                }
                break;
        }


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
                                new FollowTrajectorySequenceAsync(autoDriveSS, toCStackFromSpike),

                                new WaitCommand(100),
                                new SetIntakePower(intakeSS, -1),
                                new WaitCommand(250),
                                new SetIntakeAngle(intakeSS, INT_FIVE),
                                new WaitCommand(2000),
                                grabbersClosed,
                                new WaitCommand(250),
                                intakeIdle,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackAA),
                                armDepositGroup,
                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackAB),
                                new WaitCommand(500),
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
                                new FollowTrajectorySequenceAsync(autoDriveSS, toSpike),
                                armBackGroup,
                                new WaitCommand(500),
                                grabberLeftOpen,
                                new WaitCommand(500),
                                armGrab,
                                new WaitCommand(250),
                                new FollowTrajectorySequenceAsync(autoDriveSS, toCStackFromSpike),

                                new WaitCommand(100),
                                new SetIntakePower(intakeSS, -1),
                                new WaitCommand(250),
                                new SetIntakeAngle(intakeSS, INT_FIVE),
                                new WaitCommand(1250),
                                grabbersClosed,
                                new WaitCommand(250),
                                intakeIdle,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackAA),
                                autoArmBack,
                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackAB),
                                new WaitCommand(200),
                                pivotPosition,
                                new WaitCommand(400),
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
                                new WaitCommand(1250),
                                grabbersClosed,
                                new WaitCommand(250),
                                intakeIdle,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackBA),
                                autoArmBack,
                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackBB),
                                new WaitCommand(200),
                                pivotPosUp,
                                new WaitCommand(400),
                                grabbersOpen,
                                new WaitCommand(400),
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
