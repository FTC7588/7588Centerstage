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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.arm.ArmIdle;
import org.firstinspires.ftc.teamcode.commands.arm.ArmPoised;
import org.firstinspires.ftc.teamcode.commands.arm.AutoArmBack;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetShoulderPosition;
import org.firstinspires.ftc.teamcode.commands.drive.PIDToPoint;
import org.firstinspires.ftc.teamcode.commands.elevator.LowerElevator;
import org.firstinspires.ftc.teamcode.commands.elevator.SetElevatorPowerForTime;
import org.firstinspires.ftc.teamcode.commands.grabber.SetGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeFromStack;
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
@Disabled
public class WBlueTesting extends BaseOpMode {

    public static double xOffset = 1.2;
    public static double yOffset = -1.5;

    public static double defaultPosTol = 1;
    public static double defaultHeadingTol = 5;

    protected PIDToPoint purple;
    protected PIDToPoint offPurple;
    protected PIDToPoint stackAligned;
    protected PIDToPoint stack;
    protected PIDToPoint crossField;
    protected PIDToPoint backdropAlign1;
    protected PIDToPoint backdropAlign2;
    protected PIDToPoint backdropAlign3;
    protected PIDToPoint backdrop1;
    protected PIDToPoint backdrop2;
    protected PIDToPoint backdrop3;
    protected PIDToPoint park;

    public static Pose2d purplePose;
    public static Pose2d offPurplePose;
    public static Pose2d stackAlignedPose;
    public static Pose2d stackPose;
    public static Pose2d crossFieldPose;
    public static Pose2d backdropAlignPose1;
    public static Pose2d backdropAlignPose2;
    public static Pose2d backdropAlignPose3;
    public static Pose2d backdrop1Pose;
    public static Pose2d backdrop2Pose;
    public static Pose2d backdrop3Pose;
    public static Pose2d parkPose;

    private SequentialCommandGroup placePurple;
    private SequentialCommandGroup scoreBackdrop1;
    private SequentialCommandGroup scoreBackdrop2;
    private SequentialCommandGroup scoreBackdrop3;
    private SequentialCommandGroup scoreBackstage;
    private SequentialCommandGroup retractAndPrepare;
    private SequentialCommandGroup finish;

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
        Constants.ELE_PID = false;
        auto = false;
        alliance = Alliance.RED;
        super.initialize();

        propProcessor = new PropProcessor(Alliance.BLUE_W);

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.C920)
                .addProcessor(propProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        driveSS.setDwPose(AutoConstantsState.BlueW.START);

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
//                new SetShoulderPosition(armSS, ARM_SHOULDER_IDLE),
//                new WaitCommand(100),
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

//        proppos = propProcessor.getSpike();
        proppos = 1;

        if (gamepad1.x && !pastX) {
            visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", 1));
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
                purplePose = AutoConstantsState.BlueW.SPIKE_1;
                offPurplePose = AutoConstantsState.BlueW.SPIKE_1_BACK;
                stackAlignedPose = AutoConstantsState.BlueW.STACK_ALIGN_1;
                stackPose = AutoConstantsState.BlueW.STACK_1;
                crossFieldPose = AutoConstantsState.BlueW.INTERMEDIAN;
                backdropAlignPose1 = AutoConstantsState.BlueW.BD_1_OFF;
                backdropAlignPose2 = AutoConstantsState.BlueW.BD_2_OFF;
                backdropAlignPose3 = AutoConstantsState.BlueW.BD_3_OFF;
                backdrop1Pose = AutoConstantsState.BlueW.BD_1;
                backdrop2Pose = AutoConstantsState.BlueW.BD_2;
                backdrop3Pose = AutoConstantsState.BlueW.BD_3;
                parkPose = AutoConstantsState.BlueW.PARK;

                pivot = new InstantCommand(() -> armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.NORMAL);
                break;
            case (2):
                purplePose = AutoConstantsState.BlueW.SPIKE_2;
                offPurplePose = AutoConstantsState.BlueW.SPIKE_2_BACK;
                stackAlignedPose = AutoConstantsState.BlueW.STACK_ALIGN_1;
                stackPose = AutoConstantsState.BlueW.STACK_1;
                crossFieldPose = AutoConstantsState.BlueW.INTERMEDIAN;
                backdropAlignPose1 = AutoConstantsState.BlueW.BD_2_OFF;
                backdropAlignPose2 = AutoConstantsState.BlueW.BD_3_OFF;
                backdropAlignPose3 = AutoConstantsState.BlueW.BD_1_OFF;
                backdrop1Pose = AutoConstantsState.BlueW.BD_2;
                backdrop2Pose = AutoConstantsState.BlueW.BD_3;
                backdrop3Pose = AutoConstantsState.BlueW.BD_1;
                parkPose = AutoConstantsState.BlueW.PARK;
                pivot = new InstantCommand(() -> armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.NORMAL);
                break;
            case (3):
                purplePose = AutoConstantsState.BlueW.SPIKE_3;
                offPurplePose = AutoConstantsState.BlueW.SPIKE_3_BACK;
                stackAlignedPose = AutoConstantsState.BlueW.STACK_ALIGN_1;
                stackPose = AutoConstantsState.BlueW.STACK_1;
                crossFieldPose = AutoConstantsState.BlueW.INTERMEDIAN;
                backdropAlignPose1 = AutoConstantsState.BlueW.BD_3_OFF;
                backdropAlignPose2 = AutoConstantsState.BlueW.BD_2_OFF;
                backdropAlignPose3 = AutoConstantsState.BlueW.BD_1_OFF;
                backdrop1Pose = AutoConstantsState.BlueW.BD_3;
                backdrop2Pose = AutoConstantsState.BlueW.BD_2;
                backdrop3Pose = AutoConstantsState.BlueW.BD_1;
                parkPose = AutoConstantsState.BlueW.PARK;
                pivot = new InstantCommand(() -> armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.NORMAL);
                break;
        }



        purple = new PIDToPoint(driveSS, purplePose, 1, 5);
        offPurple = new PIDToPoint(driveSS, offPurplePose, 1.25, 5);
        stackAligned = new PIDToPoint(driveSS, stackAlignedPose, 2, 5);
        stack = new PIDToPoint(driveSS, stackPose, 1, 5);
        crossField = new PIDToPoint(driveSS, crossFieldPose, 5, 5);
        backdrop1 = new PIDToPoint(driveSS, backdrop1Pose, 1, 5);
        backdrop2 = new PIDToPoint(driveSS, backdrop2Pose, 1, 5);
        backdrop3 = new PIDToPoint(driveSS, backdrop3Pose, 1, 5);
        park = new PIDToPoint(driveSS, parkPose, 1, 5);


        placePurple = new SequentialCommandGroup(purple, offPurple);

        scoreBackdrop1 = new SequentialCommandGroup(
                stackAligned,
                stack,
                new IntakeFromStack(intakeSS, 5, 5),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new PIDToPoint(driveSS, crossFieldPose, 5, defaultHeadingTol),
                        new SequentialCommandGroup(
                                new ArmPoised(armSS),
                                new WaitCommand(200),
                                new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED),
                                new SetIntakePower(intakeSS, 1),
                                new WaitCommand(400),
                                new SetIntakePower(intakeSS, 0)
                        )
                ),

                new ParallelCommandGroup(
                        new PIDToPoint(driveSS, backdropAlignPose1, defaultPosTol, defaultHeadingTol),
                        new AutoArmBack(armSS),
                        new SetElevatorPowerForTime(eleSS, -1, 200)
                ),

                new PIDToPoint(driveSS, backdrop1Pose, defaultPosTol, defaultHeadingTol).withTimeout(400),
                new SetGrabberPosition(grabSS, GRABBER_ONE_OPEN, GRABBER_TWO_OPEN),

                new PIDToPoint(driveSS, backdropAlignPose1, defaultPosTol, defaultHeadingTol)
        );

        scoreBackdrop2 = new SequentialCommandGroup(
                new PIDToPoint(driveSS, stackAlignedPose, 5, 5),
                new PIDToPoint(driveSS, stackPose, defaultPosTol, defaultHeadingTol),

                //intake at stack
                new IntakeFromStack(intakeSS, 4, 5),
                new WaitCommand(400),
                new ArmPoised(armSS),
                new WaitCommand(200),
                new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED),

                //run away and eject
                new SetIntakePower(intakeSS, 1),
                new PIDToPoint(driveSS, crossFieldPose, 5, defaultPosTol),

                //align with backdrop
                new ParallelCommandGroup(
                        new PIDToPoint(driveSS, backdropAlignPose2, defaultPosTol, defaultHeadingTol),
                        new SequentialCommandGroup(
                                new SetIntakePower(intakeSS, 0),
                                new WaitCommand(250),
                                new AutoArmBack(armSS),
                                new SetElevatorPowerForTime(eleSS, -1, 200)
                        )
                ),

                //shtuff
                new PIDToPoint(driveSS, backdrop2Pose, 1, 5).withTimeout(400),
                new SetGrabberPosition(grabSS, GRABBER_ONE_OPEN, GRABBER_TWO_OPEN),

                new PIDToPoint(driveSS, backdropAlignPose2, defaultPosTol, defaultHeadingTol).withTimeout(400)
        );

        scoreBackdrop3 = new SequentialCommandGroup(
                new PIDToPoint(driveSS, stackAlignedPose, 5, 5),
                new PIDToPoint(driveSS, stackPose, defaultPosTol, defaultHeadingTol),

                //intake at stack
                new IntakeFromStack(intakeSS, 4, 5),
                new WaitCommand(400),
                new ArmPoised(armSS),
                new WaitCommand(200),
                new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED),

                //run away and eject
                new SetIntakePower(intakeSS, 1),
                new PIDToPoint(driveSS, crossFieldPose, 5, defaultPosTol),

                //align with backdrop
                new ParallelCommandGroup(
                        new PIDToPoint(driveSS, backdropAlignPose2, defaultPosTol, defaultHeadingTol),
                        new SequentialCommandGroup(
                                new SetIntakePower(intakeSS, 0),
                                new WaitCommand(250),
                                new AutoArmBack(armSS),
                                new SetElevatorPowerForTime(eleSS, -1, 200)
                        )
                ),

                //shtuff
                new PIDToPoint(driveSS, backdrop3Pose, 1, 5).withTimeout(400),
                new SetGrabberPosition(grabSS, GRABBER_ONE_OPEN, GRABBER_TWO_OPEN),

                new PIDToPoint(driveSS, backdropAlignPose3, defaultPosTol, defaultHeadingTol).withTimeout(400)
        );

        scoreBackstage = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PIDToPoint(driveSS, crossFieldPose, 5, defaultPosTol),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new ArmIdle(armSS),
                                new LowerElevator(eleSS),
                                new InstantCommand(() -> driveSS.setDwPose(new Pose2d(driveSS.getDwPose().x + xOffset, driveSS.getDwPose().y + yOffset, driveSS.getDwPose().theta)))
                        )
                ),

                new PIDToPoint(driveSS, stackAlignedPose, 5, 5),
                new PIDToPoint(driveSS, stackPose, defaultPosTol, defaultHeadingTol),

                //intake at stack
                new IntakeFromStack(intakeSS, 2, 3),
                new WaitCommand(400),
                new ArmPoised(armSS),
                new WaitCommand(200),
                new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED),

                //run away and eject
                new SetIntakePower(intakeSS, 1),
                new PIDToPoint(driveSS, crossFieldPose, 5, defaultPosTol),

                //align with backdrop
                new ParallelCommandGroup(
                        new PIDToPoint(driveSS, parkPose, defaultPosTol, defaultHeadingTol),
                        new SequentialCommandGroup(
                                new SetIntakePower(intakeSS, 0),
                                new WaitCommand(250),
                                new SetElevatorPowerForTime(eleSS, -1, 300),
                                new WaitCommand(100),
                                new AutoArmBack(armSS)
                        )
                ),

                //shtuff
                new SetGrabberPosition(grabSS, GRABBER_ONE_OPEN, GRABBER_TWO_OPEN)
        );

        retractAndPrepare = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PIDToPoint(driveSS, crossFieldPose, 5, defaultPosTol),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new ArmIdle(armSS),
                                new LowerElevator(eleSS),
                                new InstantCommand(() -> driveSS.setDwPose(new Pose2d(driveSS.getDwPose().x + xOffset, driveSS.getDwPose().y + yOffset, driveSS.getDwPose().theta)))
                        )
                )
        );

        finish = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PIDToPoint(driveSS, parkPose, defaultPosTol, defaultHeadingTol),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new ArmIdle(armSS),
                                new LowerElevator(eleSS)
                        )
                )
        );

        schedule(
                new SequentialCommandGroup(
                        placePurple,
                        scoreBackdrop1,
                        retractAndPrepare,
                        scoreBackdrop2,
                        retractAndPrepare,
                        scoreBackstage,
                        finish
                )
        );


//        schedule(new SequentialCommandGroup(
//                purple,
//                offPurple,
//                stackAligned,
//                stack,
//
//                new IntakeFromStack(intakeSS, 5, 5),
//                new WaitCommand(100),
//                new ParallelCommandGroup(
//                        new PIDToPoint(driveSS, crossFieldPose, 5, defaultHeadingTol),
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
//                new ParallelCommandGroup(
//                        new PIDToPoint(driveSS, backdropAlignPose1, defaultPosTol, defaultHeadingTol),
//                        new AutoArmBack(armSS),
//                        new SetElevatorPowerForTime(eleSS, -1, 200)
//                ),
//
//                backdrop1.withTimeout(400),
//                new SetGrabberPosition(grabSS, GRABBER_ONE_OPEN, GRABBER_TWO_OPEN),
//
//                new PIDToPoint(driveSS, backdropAlignPose1, defaultPosTol, defaultHeadingTol),
//
//                new ParallelCommandGroup(
//                        new PIDToPoint(driveSS, crossFieldPose, 5, defaultPosTol),
//                        new SequentialCommandGroup(
//                                new WaitCommand(100),
//                                new ArmIdle(armSS),
//                                new LowerElevator(eleSS),
//                                new InstantCommand(() -> driveSS.setDwPose(new Pose2d(driveSS.getDwPose().x + xOffset, driveSS.getDwPose().y + yOffset, driveSS.getDwPose().theta)))
//                        )
//                ),
//
//            //cycle 2
//                new PIDToPoint(driveSS, stackAlignedPose, 5, 5),
//                new PIDToPoint(driveSS, stackPose, defaultPosTol, defaultHeadingTol),
//
//                //intake at stack
//                new IntakeFromStack(intakeSS, 4, 5),
//                new WaitCommand(400),
//                new ArmPoised(armSS),
//                new WaitCommand(200),
//                new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED),
//
//                //run away and eject
//                new SetIntakePower(intakeSS, 1),
//                new PIDToPoint(driveSS, crossFieldPose, 5, defaultPosTol),
//
//                //align with backdrop
//                new ParallelCommandGroup(
//                        new PIDToPoint(driveSS, backdropAlignPose2, defaultPosTol, defaultHeadingTol),
//                        new SequentialCommandGroup(
//                                new SetIntakePower(intakeSS, 0),
//                                new WaitCommand(250),
//                                new AutoArmBack(armSS),
//                                new SetElevatorPowerForTime(eleSS, -1, 200)
//                        )
//                ),
//
//                //shtuff
//                new PIDToPoint(driveSS, backdrop2Pose, 1, 5).withTimeout(400),
//                new SetGrabberPosition(grabSS, GRABBER_ONE_OPEN, GRABBER_TWO_OPEN),
//
//                //cycle 3
//                new ParallelCommandGroup(
//                        new PIDToPoint(driveSS, crossFieldPose, 5, defaultPosTol),
//                        new SequentialCommandGroup(
//                                new WaitCommand(100),
//                                new ArmIdle(armSS),
//                                new LowerElevator(eleSS),
//                                new InstantCommand(() -> driveSS.setDwPose(new Pose2d(driveSS.getDwPose().x + xOffset, driveSS.getDwPose().y + yOffset, driveSS.getDwPose().theta)))
//                        )
//                ),
//
//                new PIDToPoint(driveSS, stackAlignedPose, 5, 5),
//                new PIDToPoint(driveSS, stackPose, defaultPosTol, defaultHeadingTol),
//
//                //intake at stack
//                new IntakeFromStack(intakeSS, 2, 3),
//                new WaitCommand(400),
//                new ArmPoised(armSS),
//                new WaitCommand(200),
//                new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED),
//
//                //run away and eject
//                new SetIntakePower(intakeSS, 1),
//                new PIDToPoint(driveSS, crossFieldPose, 5, defaultPosTol),
//
//                //align with backdrop
//                new ParallelCommandGroup(
//                        new PIDToPoint(driveSS, parkPose, defaultPosTol, defaultHeadingTol),
//                        new SequentialCommandGroup(
//                                new SetIntakePower(intakeSS, 0),
//                                new WaitCommand(250),
//                                new SetElevatorPowerForTime(eleSS, -1, 300),
//                                new WaitCommand(100),
//                                new AutoArmBack(armSS)
//                        )
//                ),
//
//                //shtuff
//                new SetGrabberPosition(grabSS, GRABBER_ONE_OPEN, GRABBER_TWO_OPEN),
//                new WaitCommand(100),
//                new ArmIdle(armSS),
//                new LowerElevator(eleSS)
//        ));
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
