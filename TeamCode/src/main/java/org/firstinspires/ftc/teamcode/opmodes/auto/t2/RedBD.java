package org.firstinspires.ftc.teamcode.opmodes.auto.t2;

import static org.firstinspires.ftc.teamcode.Constants.INT_FIVE;
import static org.firstinspires.ftc.teamcode.Constants.INT_FOUR;
import static org.firstinspires.ftc.teamcode.opmodes.auto.t2.AutoConstants.Red.*;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
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

@Autonomous
@Config
public class RedBD extends BaseOpMode {

    public static Paths path = Paths.BS_0;
    private int list = 0;

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    private final double WAIT_BD = 1;
    private final double WAIT_SPIKE = 1;

    private TrajectorySequence toSpike;
    private TrajectorySequence toBDFromSpike;
    private TrajectorySequence toCStackFromBD;
    private TrajectorySequence toBDFromCStackA;
    private TrajectorySequence toBDFromCStackB;
    private TrajectorySequence park;

    public static int propPos = 1;

    private boolean pastA = false;

    @Override
    public void initialize() {
        Constants.ELE_PID = false;
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
        grabberLeftOpen.schedule();
        grabberRightClose.schedule();

        robot.read(intakeSS, eleSS, armSS, grabSS);

        robot.loop(intakeSS, eleSS, armSS, grabSS);

        robot.write(intakeSS, eleSS, armSS, grabSS);

//        propPos = propProcessor.getSpike();

        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", 1));
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

        switch (path) {
            case BS_0:
                switch (propPos) {
                    case 1:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(BD_START)
                                .lineToLinearHeading(BD_SPIKE_ONE)
                                .build();

                        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(BD_BD_ONE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                                .lineToLinearHeading(PARK_CORNER)
                                .build();
                        break;
                    case 2:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(BD_START)
                                .lineToLinearHeading(BD_SPIKE_TWO)
                                .build();

                        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(BD_BD_TWO_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                                .lineToLinearHeading(PARK_CORNER)
                                .build();
                        break;
                    case 3:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(BD_START)
                                .lineToLinearHeading(BD_SPIKE_THREE)
                                .build();

                        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(BD_BD_THREE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                                .lineToLinearHeading(PARK_CORNER)
                                .build();
                        break;
                }
                break;
            case BS_2_A:
                switch (propPos) {
                    case 1:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(BD_START)
                                .lineToLinearHeading(BD_SPIKE_ONE)
                                .build();

                        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(BD_BD_ONE_OFF)
                                .build();

                        toCStackFromBD = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                                .lineToLinearHeading(STACK_BD_2_A)
                                .lineToLinearHeading(STACK_BD_2_B)
                                .lineToLinearHeading(STACK_A)
                                .build();

                        toBDFromCStackA = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD.end())
                                .lineToLinearHeading(STACK_BD_2_B)
                                .lineToLinearHeading(STACK_BD_2_A)
                                .build();

                        toBDFromCStackB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackA.end())
                                .lineToLinearHeading(BD_BD_TWO_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackB.end())
                                .lineToLinearHeading(PARK_CORNER)
                                .build();
                        break;
                    case 2:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(BD_START)
                                .lineToLinearHeading(BD_SPIKE_TWO)
                                .build();

                        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(BD_BD_TWO_OFF)
                                .build();

                        toCStackFromBD = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                                .lineToLinearHeading(STACK_BD_2_A)
                                .lineToLinearHeading(STACK_BD_2_B)
                                .lineToLinearHeading(STACK_A)
                                .build();

                        toBDFromCStackA = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD.end())
                                .lineToLinearHeading(STACK_BD_2_B)
                                .lineToLinearHeading(STACK_BD_2_A)
                                .build();

                        toBDFromCStackB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackA.end())
                                .lineToLinearHeading(BD_BD_THREE_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackB.end())
                                .lineToLinearHeading(PARK_CORNER)
                                .build();
                        break;
                    case 3:
                        toSpike = autoDriveSS.trajectorySequenceBuilder(BD_START)
                                .lineToLinearHeading(BD_SPIKE_THREE)
                                .build();

                        toBDFromSpike = autoDriveSS.trajectorySequenceBuilder(toSpike.end())
                                .lineToLinearHeading(BD_BD_THREE_OFF)
                                .build();

                        toCStackFromBD = autoDriveSS.trajectorySequenceBuilder(toBDFromSpike.end())
                                .lineToLinearHeading(STACK_BD_2_A)
                                .lineToLinearHeading(STACK_BD_2_B)
                                .lineToLinearHeading(STACK_A)
                                .build();

                        toBDFromCStackA = autoDriveSS.trajectorySequenceBuilder(toCStackFromBD.end())
                                .lineToLinearHeading(STACK_BD_2_B)
                                .lineToLinearHeading(STACK_BD_2_A)
                                .build();

                        toBDFromCStackB = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackA.end())
                                .lineToLinearHeading(BD_BD_TWO_OFF)
                                .build();

                        park = autoDriveSS.trajectorySequenceBuilder(toBDFromCStackB.end())
                                .lineToLinearHeading(PARK_CORNER)
                                .build();
                        break;
                }
        }


        switch (path) {
            case BS_0:
                schedule(
                        new SequentialCommandGroup(
                                new FollowTrajectorySequenceAsync(autoDriveSS, toSpike),
                                new SetIntakePower(intakeSS, 0.7),
                                new WaitCommand(500),
                                intakeIdle,
                                new WaitCommand(250),
                                autoArmBack,
                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromSpike),
                                grabberRightOpen,
                                new WaitCommand(500),
                                armGrab,
                                new FollowTrajectorySequenceAsync(autoDriveSS, park)
                        )
                );
                break;
            case BS_2_A:
                schedule(
                        new SequentialCommandGroup(
                                new FollowTrajectorySequenceAsync(autoDriveSS, toSpike),
                                new SetIntakePower(intakeSS, 0.7),
                                new WaitCommand(500),
                                intakeIdle,
                                autoArmBack,
                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromSpike),
                                grabberRightOpen,
                                new WaitCommand(500),
                                armGrab,
                                new FollowTrajectorySequenceAsync(autoDriveSS, toCStackFromBD),
                                new WaitCommand(100),
                                new SetIntakePower(intakeSS, -1),
                                new WaitCommand(250),
                                new SetIntakeAngle(intakeSS, INT_FIVE),
                                new WaitCommand(1000),
                                new SetIntakeAngle(intakeSS, INT_FOUR),
                                new WaitCommand(1500),
                                grabbersClosed,
                                new WaitCommand(250),
                                intakeIdle,

                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackA),
                                autoArmBack,
                                new FollowTrajectorySequenceAsync(autoDriveSS, toBDFromCStackB),
                                new WaitCommand(250),
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
