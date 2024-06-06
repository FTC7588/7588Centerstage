package org.firstinspires.ftc.teamcode.opmodes.auto.state;

import static org.firstinspires.ftc.teamcode.Constants.ARM_PIVOT_MID;
import static org.firstinspires.ftc.teamcode.Constants.GRABBER_ONE_CLOSED;
import static org.firstinspires.ftc.teamcode.Constants.GRABBER_ONE_OPEN;
import static org.firstinspires.ftc.teamcode.Constants.GRABBER_TWO_CLOSED;
import static org.firstinspires.ftc.teamcode.Constants.GRABBER_TWO_OPEN;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_SHOULDER;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_WRIST;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.arm.ArmIdle;
import org.firstinspires.ftc.teamcode.commands.arm.ArmPoised;
import org.firstinspires.ftc.teamcode.commands.arm.AutoArmBack;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetShoulderPosition;
import org.firstinspires.ftc.teamcode.commands.drive.PIDToPoint;
import org.firstinspires.ftc.teamcode.commands.drive.PIDToPointSpeed;
import org.firstinspires.ftc.teamcode.commands.elevator.LowerElevator;
import org.firstinspires.ftc.teamcode.commands.elevator.SetElevatorPowerForTime;
import org.firstinspires.ftc.teamcode.commands.grabber.SetGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeFromStack;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.poofyutils.PoofyDashboardUtil;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Config
@Autonomous
public class BDRedAuto extends BaseOpMode {

    public static double xOffset = 1.2;
    public static double yOffset = -1.5;

    public static double defaultPosTol = 1;
    public static double defaultHeadingTol = 5;
    public static double slowSpeed = 0.75;

    protected PIDToPoint purple;
    protected PIDToPoint prePurple;
    protected PIDToPoint offPurple;
    protected PIDToPoint backdropAlign1;
    protected PIDToPoint backdropAlign2;
    protected PIDToPoint backdropAlign3;
    protected PIDToPoint backdrop1;
    protected PIDToPoint backdrop2;
    protected PIDToPoint backdrop3;
    protected PIDToPoint stackAligned;
    protected PIDToPoint stack;
    protected PIDToPoint crossFieldBD;
    protected PIDToPoint crossFieldW;
    protected PIDToPoint park;

    public static Pose2d purplePose;
    public static Pose2d prePurplePose;
    public static Pose2d offPurplePose;
    public static Pose2d stackAlignedPose;
    public static Pose2d stackPose;
    public static Pose2d crossFieldPoseBD;
    public static Pose2d crossFieldPoseW;
    public static Pose2d backdropAlignPose1;
    public static Pose2d backdropAlignPose2;
    public static Pose2d backdropAlignPose3;
    public static Pose2d backdrop1Pose;
    public static Pose2d backdrop2Pose;
    public static Pose2d backdrop3Pose;
    public static Pose2d parkPose;

    private InstantCommand pivot;

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    private ElapsedTime time;

    public int proppos;

    public boolean pastX = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        RobotHardware.USING_TAGS = false;
        RobotHardware.USING_IMU = true;
        RobotHardware.USING_SENSORS = true;
        Constants.ELE_PID = false;
        auto = false;
        alliance = Alliance.RED;
        super.initialize();

        propProcessor = new PropProcessor(Alliance.RED_BD);

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.C920)
                .addProcessor(propProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        driveSS.setDwPose(AutoConstantsState.RedBD.START);

        time = new ElapsedTime();

        new SequentialCommandGroup(
                new SetArmPositions(
                        armSS,
                        GRAB_SHOULDER,
                        GRAB_WRIST,
                        ARM_PIVOT_MID
                ),
                new InstantCommand(() -> {
                    armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.NORMAL;
                    armSS.pivotPositionState = ArmSubsystem.PivotPositionState.UP;
                }),
                new SetShoulderPosition(armSS, GRAB_SHOULDER)
        ).schedule();
        grabberLeftOpen.schedule();
        grabberRightOpen.schedule();
        intakeUp.schedule();
    }

    @Override
    public void initLoop() {
        robot.read(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.loop(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.write(driveSS, intakeSS, eleSS, armSS, grabSS);

        CommandScheduler.getInstance().run();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING);

        proppos = propProcessor.getSpikePast();
//        proppos = 3;

        if (gamepad1.x && !pastX) {
            visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCaptureRedBD-%06d", System.currentTimeMillis()));
        }
        pastX = gamepad1.x;

        time.reset();

        telemetry.addData("spike pos", proppos);
        telemetry.update();
    }

    @Override
    public void runOnce() {
        switch (proppos) {
            case (1):
                purplePose = AutoConstantsState.RedBD.SPIKE_1;
                prePurplePose = AutoConstantsState.RedBD.SPIKE_1_PREP;
                offPurplePose = AutoConstantsState.RedBD.SPIKE_1_BACK;
                stackAlignedPose = AutoConstantsState.RedBD.STACK_ALIGNED;
                stackPose = AutoConstantsState.RedBD.STACK;
                crossFieldPoseBD = AutoConstantsState.RedBD.CROSS_FIELD_BD;
                crossFieldPoseW = AutoConstantsState.RedBD.CROSS_FIELD_W;
                backdropAlignPose1 = AutoConstantsState.RedBD.BD_1_OFF;
                backdropAlignPose2 = AutoConstantsState.RedBD.BD_2_OFF;
                backdropAlignPose3 = AutoConstantsState.RedBD.BD_3_OFF;
                backdrop1Pose = AutoConstantsState.RedBD.BD_1;
                backdrop2Pose = AutoConstantsState.RedBD.BD_2;
                backdrop3Pose = AutoConstantsState.RedBD.BD_3;
                parkPose = AutoConstantsState.RedBD.PARK;
                pivot = new InstantCommand(() -> armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.NORMAL);
                slowSpeed = 0.75;
                xOffset = 0;
                yOffset = 1;
                break;
            case (2):
                purplePose = AutoConstantsState.RedBD.SPIKE_2;
                prePurplePose = AutoConstantsState.RedBD.SPIKE_2_PREP;
                offPurplePose = AutoConstantsState.RedBD.SPIKE_2_BACK;
                stackAlignedPose = AutoConstantsState.RedBD.STACK_ALIGNED;
                stackPose = AutoConstantsState.RedBD.STACK;
                crossFieldPoseBD = AutoConstantsState.RedBD.CROSS_FIELD_BD;
                crossFieldPoseW = AutoConstantsState.RedBD.CROSS_FIELD_W;
                backdropAlignPose1 = AutoConstantsState.RedBD.BD_2_OFF;
                backdropAlignPose2 = AutoConstantsState.RedBD.BD_3_OFF;
                backdropAlignPose3 = AutoConstantsState.RedBD.BD_1_OFF;
                backdrop1Pose = AutoConstantsState.RedBD.BD_2;
                backdrop2Pose = AutoConstantsState.RedBD.BD_3;
                backdrop3Pose = AutoConstantsState.RedBD.BD_1;
                parkPose = AutoConstantsState.RedBD.PARK;
                pivot = new InstantCommand(() -> armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.NORMAL);
                slowSpeed = 0.625;
                xOffset = 0;
                yOffset = -1.5;
                break;
            case (3):
                purplePose = AutoConstantsState.RedBD.SPIKE_3;
                prePurplePose = AutoConstantsState.RedBD.SPIKE_3_PREP;
                offPurplePose = AutoConstantsState.RedBD.SPIKE_3_BACK;
                stackAlignedPose = AutoConstantsState.RedBD.STACK_ALIGNED;
                stackPose = AutoConstantsState.RedBD.STACK;
                crossFieldPoseBD = AutoConstantsState.RedBD.CROSS_FIELD_BD;
                crossFieldPoseW = AutoConstantsState.RedBD.CROSS_FIELD_W;
                backdropAlignPose1 = AutoConstantsState.RedBD.BD_3_OFF;
                backdropAlignPose2 = AutoConstantsState.RedBD.BD_1_OFF;
                backdropAlignPose3 = AutoConstantsState.RedBD.BD_2_OFF;
                backdrop1Pose = AutoConstantsState.RedBD.BD_3;
                backdrop2Pose = AutoConstantsState.RedBD.BD_1;
                backdrop3Pose = AutoConstantsState.RedBD.BD_2;
                parkPose = AutoConstantsState.RedBD.PARK;
                pivot = new InstantCommand(() -> armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.ROTATED);
                slowSpeed = 0.75;
                xOffset = 0;
                yOffset = -0.5;
                break;
        }

//        schedule(new SequentialCommandGroup(
//                new PIDToPointSpeed(driveSS, prePurplePose, 1.25, defaultHeadingTol, slowSpeed),
//                new PIDToPoint(driveSS, purplePose, 1.25, defaultHeadingTol),
//                new PIDToPoint(driveSS, offPurplePose, 3, 10),
//                new SetIntakeAngle(intakeSS, Constants.INT_UP),
//
//                new PIDToPoint(driveSS, backdropAlignPose1, defaultPosTol, defaultHeadingTol),
//                new PIDToPoint(driveSS, backdrop1Pose, defaultPosTol, defaultHeadingTol).withTimeout(400),
//                new PIDToPoint(driveSS, backdropAlignPose1, defaultPosTol, defaultHeadingTol),
//
//                new InstantCommand(() -> driveSS.setDwPose(new Pose2d(driveSS.getDwPose().x + xOffset, driveSS.getDwPose().y + yOffset, driveSS.getDwPose().theta))),
//
//                new PIDToPoint(driveSS, crossFieldPoseBD, defaultPosTol, defaultHeadingTol),
//                new PIDToPoint(driveSS, crossFieldPoseW, defaultPosTol, defaultHeadingTol),
//
//                new PIDToPoint(driveSS, stackAlignedPose, defaultPosTol, defaultHeadingTol),
//                new PIDToPoint(driveSS, stackPose, defaultPosTol, defaultHeadingTol).withTimeout(1000),
//
//                new PIDToPoint(driveSS, crossFieldPoseW, defaultPosTol, defaultHeadingTol),
//                new PIDToPoint(driveSS, crossFieldPoseBD, defaultPosTol, defaultHeadingTol),
//
//                new PIDToPoint(driveSS, backdropAlignPose2, defaultPosTol, defaultHeadingTol),
//                new PIDToPoint(driveSS, backdrop2Pose, defaultPosTol, defaultHeadingTol),
//                new PIDToPoint(driveSS, backdropAlignPose2, defaultPosTol, defaultHeadingTol),
//
//                new InstantCommand(() -> driveSS.setDwPose(new Pose2d(driveSS.getDwPose().x + xOffset, driveSS.getDwPose().y + yOffset, driveSS.getDwPose().theta))),
//
//                new PIDToPoint(driveSS, parkPose, defaultPosTol, defaultHeadingTol)
//
//        ));


        schedule(new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new ArmPoised(armSS),
                        new WaitCommand(200),
                        new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED)
                ),
                new PIDToPointSpeed(driveSS, prePurplePose, 1.25, defaultHeadingTol, slowSpeed),
                new PIDToPoint(driveSS, purplePose, 1.25, defaultHeadingTol),
                new PIDToPoint(driveSS, offPurplePose, 3, 10),
                new SetIntakeAngle(intakeSS, Constants.INT_UP),

                new ParallelCommandGroup(
                        new PIDToPointSpeed(driveSS, backdropAlignPose1, defaultPosTol, defaultHeadingTol, 0.75),
                        new AutoArmBack(armSS),
                        new WaitCommand(100),
                        pivot,
                        new SetElevatorPowerForTime(eleSS, -1, 50)
                ),

                new WaitCommand(250),

                new PIDToPoint(driveSS, backdrop1Pose, defaultPosTol, defaultHeadingTol).withTimeout(400),
                new SetGrabberPosition(grabSS, GRABBER_ONE_OPEN, GRABBER_TWO_OPEN),

                new WaitCommand(100),

                new PIDToPoint(driveSS, backdropAlignPose1, defaultPosTol, defaultHeadingTol),

                new InstantCommand(() -> driveSS.setDwPose(new Pose2d(driveSS.getDwPose().x + xOffset, driveSS.getDwPose().y + yOffset, driveSS.getDwPose().theta))),

                new PIDToPoint(driveSS, parkPose, defaultPosTol, defaultHeadingTol),
                    new SequentialCommandGroup(
                            new WaitCommand(100),
                            new ArmIdle(armSS),
                            new LowerElevator(eleSS)
                    )
                //cycle 2
//                new ParallelCommandGroup(
//                        new PIDToPointSpeed(driveSS, crossFieldPoseBD, 3, defaultHeadingTol, 0.75),
//                        new SequentialCommandGroup(
//                                new WaitCommand(100),
//                                new ArmIdle(armSS),
//                                new LowerElevator(eleSS)
//                        )
//                ),
//
////                new PIDToPoint(driveSS, crossFieldPoseBD, defaultPosTol, defaultHeadingTol),
//                new PIDToPointSpeed(driveSS, crossFieldPoseW, 3, defaultHeadingTol, 0.75),
//
//                new PIDToPoint(driveSS, stackAlignedPose, defaultPosTol, defaultHeadingTol),
//                new PIDToPoint(driveSS, stackPose, defaultPosTol, defaultHeadingTol).withTimeout(1000),
//
//                new IntakeFromStack(intakeSS, 5, 5),
//                new WaitCommand(100),
//
//                new ParallelCommandGroup(
//                        new PIDToPoint(driveSS, crossFieldPoseW, defaultPosTol, defaultHeadingTol),
//                        new SequentialCommandGroup(
//                                new ArmPoised(armSS),
//                                new WaitCommand(200),
//                                new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED),
//                                new SetIntakePower(intakeSS, 1),
//                                new WaitCommand(400),
//                                new SetIntakePower(intakeSS, 0)
//                        )
//                ),
//
//                new PIDToPoint(driveSS, crossFieldPoseBD, defaultPosTol, defaultHeadingTol),
//
//                new ParallelCommandGroup(
//                        new PIDToPoint(driveSS, backdropAlignPose2, defaultPosTol, defaultHeadingTol),
//                        new SequentialCommandGroup(
//                                new AutoArmBack(armSS),
//                                new WaitCommand(300),
//                                new SetElevatorPowerForTime(eleSS, -1, 400)
//                        )
//                ),
//
//                new PIDToPoint(driveSS, backdrop2Pose, defaultPosTol, defaultHeadingTol).withTimeout(500),
//                grabbersOpen,
//
//                new PIDToPoint(driveSS, backdropAlignPose2, defaultPosTol, defaultHeadingTol),
//
//                new InstantCommand(() -> driveSS.setDwPose(new Pose2d(driveSS.getDwPose().x + xOffset, driveSS.getDwPose().y + yOffset, driveSS.getDwPose().theta))),
//
//                new ParallelCommandGroup(
//                        new PIDToPoint(driveSS, parkPose, defaultPosTol, defaultHeadingTol),
//                        new ParallelCommandGroup(
//                                new ArmIdle(armSS),
//                                new LowerElevator(eleSS)
//                        )
//                )

        ));
    }

    @Override
    public void run() {
        super.run();

        robot.read(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.loop(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.write(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.clearBulkCache();

        tad("target", driveSS.getTargetPose());
        telemetry.addData("target pose", driveSS.getTargetPose());
        telemetry.addData("spike", proppos);
        telemetry.addData("current", driveSS.getDwPose());
        tal();
        tad("reachedd x", driveSS.drive.reachedXTarget(0.5, driveSS.getDwPose()));
        tad("reachedd y", driveSS.drive.reachedYTarget(0.5, driveSS.getDwPose()));
        tad("reachedd theta", driveSS.drive.reachedThetaTarget(2, driveSS.getDwPose()));

        tad("intake current", intakeSS.getAverageCurrent());

        TelemetryPacket packet = new TelemetryPacket();

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getDwPose());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getTargetPose());

        dashboard.sendTelemetryPacket(packet);

        tau();
    }
}
