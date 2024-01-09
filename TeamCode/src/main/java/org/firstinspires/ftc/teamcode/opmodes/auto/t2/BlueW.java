package org.firstinspires.ftc.teamcode.opmodes.auto.t2;

import static org.firstinspires.ftc.teamcode.opmodes.auto.t2.AutoConstants.Blue.*;

import android.util.Size;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commands.elevator.SetElevatorTarget;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.RunDuringDelayCommand;
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

    private TrajectorySequence toCStackFromSpike;

    private TrajectorySequence toCStackFromBD;

    private TrajectorySequence toBDFromCStackA;
    private TrajectorySequence toBDFromCStackB;

    private TrajectorySequence toBDFromSpike;

    private TrajectorySequence park;

    private int propPos;

    private boolean pastA = false;

    private double eleHeight;

    @Override
    public void initialize() {
        Constants.ELE_PID = false;
        auto = true;
        super.initialize();

        eleHeight = 600;

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
        new SetIntakeAngle(intakeSS, INT_UP).schedule();
        armGrab.schedule();
        grabberLeftClose.schedule();
        grabberRightOpen.schedule();
//        grabbersClosed.schedule();

        robot.read(intakeSS, eleSS, armSS, grabSS);

        robot.loop(intakeSS, eleSS, armSS, grabSS);

        robot.write(intakeSS, eleSS, armSS, grabSS);

        propPos = propProcessor.getSpike();

        //take picture
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", 1));
            tal("Picture Taken!");
        }

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
                        break;
                    case 2:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_TWO)
                                .build();

                        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(W_BD_TWO_A)
                                .lineToLinearHeading(W_BD_TWO_B)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
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
                                .lineToLinearHeading(STACK_C)
                                .build();

                        toBDFromCStackA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(BD_ONE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackA.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        break;
                    case 2:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_TWO)
                                .build();

                        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(STACK_C)
                                .build();

                        toBDFromCStackA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(BD_TWO_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackA.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        break;
                    case 3:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_THREE)
                                .build();

                        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(STACK_C)
                                .build();

                        toBDFromCStackA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(BD_THREE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackA.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
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
                                .lineToLinearHeading(STACK_C)
                                .build();

                        toBDFromCStackA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(BD_ONE_OFF)
                                .build();

                        toCStackFromBD = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackA.end())
                                .setReversed(false)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(STACK_C)
                                .build();

                        toBDFromCStackB = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(BD_TWO_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackB.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        break;
                    case 2:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_TWO)
                                .build();

                        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(STACK_C)
                                .build();

                        toBDFromCStackA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(BD_TWO_OFF)
                                .build();

                        toCStackFromBD = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackA.end())
                                .setReversed(false)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(STACK_C)
                                .build();

                        toBDFromCStackB = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(BD_ONE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackB.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        break;
                    case 3:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
                                .lineToLinearHeading(W_SPIKE_THREE)
                                .build();

                        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(STACK_C)
                                .build();

                        toBDFromCStackA = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(BD_THREE_OFF)
                                .build();

                        toCStackFromBD = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackA.end())
                                .setReversed(false)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(STACK_C)
                                .build();

                        toBDFromCStackB = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD.end())
                                .setReversed(true)
                                .lineToLinearHeading(STACK_BD_1_A)
                                .lineToLinearHeading(BD_TWO_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackB.end())
                                .lineToLinearHeading(PARK_CENTER)
                                .build();
                        break;
                }
                break;
        }



//        toSpike = autoDriveSS.trajectorySequenceBuilder(W_START)
//                .lineToLinearHeading(W_SPIKE_ONE)
//                .build();
//
//        toCStackFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
//                .lineToLinearHeading(STACK_C)
//                .build();
//
//        toBD1FromCStack = autoDriveSS.trajectorySequenceBuilder(toCStackFromSpike.end())
//                .setReversed(true)
//                .lineToLinearHeading(STACK_BD_1_A)
//                .lineToLinearHeading(BD_ONE_OFF)
//                .build();
//
//        toBD2FromCStack = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD1.end())
//                .setReversed(true)
//                .lineToLinearHeading(STACK_BD_1_A)
//                .lineToLinearHeading(BD_TWO_OFF)
//                .build();
//
//        toCStackFromBD1 = autoDriveSS.trajectorySequenceBuilder(toBD1FromCStack.end())
//                .setReversed(false)
//                .lineToLinearHeading(STACK_BD_1_A)
//                .lineToLinearHeading(STACK_C)
//                .build();
//
//        toBD1FromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
//                .lineToLinearHeading(W_BD_ONE_A)
//                .splineToLinearHeading(W_BD_ONE_B, W_BD_ONE_B_TANGENT)
//                .splineToLinearHeading(BD_ONE_OFF, W_BD_ONE_C_TANGENT)
//                .build();
//
//        push = autoDriveSS.trajectorySequenceBuilder(toBD1FromSpike.end())
//                .lineToLinearHeading(BD_ONE_PUSH)
//                .build();
//
//        park = autoDriveSS.trajectorySequenceBuilder(push.end())
//                .lineToLinearHeading(PARK_CENTER)
//                .build();

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
                                intakeIdle,
                                new WaitCommand(500),
                                grabbersClosed,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackA),
                                armDepositGroup,
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
                                new WaitCommand(1000),
                                intakeIdle,
                                new WaitCommand(500),
                                grabbersClosed,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackA),
                                autoArmBack,
                                new WaitCommand(200),
                                pivotPosUp,
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
                                new WaitCommand(1000),
                                intakeIdle,
                                new WaitCommand(500),
                                grabbersClosed,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackB),
                                new SetElevatorTarget(eleSS, eleHeight),
                                autoArmBack,
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
