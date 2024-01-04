package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectoryAsyncCommand;
import org.firstinspires.ftc.teamcode.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.Blue.*;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueBDZero extends BaseOpMode {

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    public double WAIT_BD = 1;
    public double WAIT_SPIKE = 1;

    public TrajectorySequence toSpike;
    public TrajectorySequence toBD;
    public TrajectorySequence push;
    public TrajectorySequence park;

    public FollowTrajectorySequenceAsync cmd;

    @Override
    public void initialize() {
        auto = true;
        super.initialize();
        autoDriveSS.setPoseEstimate(BD_START);
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

        intakeUp.schedule();
        armGrab.schedule();
        grabbersClosed.schedule();

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

        telemetry.addLine("test");
        telemetry.update();
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
