package org.firstinspires.ftc.teamcode.opmodes.auto.state;

import static org.firstinspires.ftc.teamcode.Constants.ARM_PIVOT_MID;
import static org.firstinspires.ftc.teamcode.Constants.ARM_SHOULDER_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Constants.ARM_SHOULDER_IDLE;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_SHOULDER;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_WRIST;
import static org.firstinspires.ftc.teamcode.Constants.INT_THREE;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.arm.AutoArmBack;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetShoulderPosition;
import org.firstinspires.ftc.teamcode.commands.drive.PIDToPoint;
import org.firstinspires.ftc.teamcode.commands.elevator.LowerElevator;
import org.firstinspires.ftc.teamcode.commands.elevator.SetElevatorPower;
import org.firstinspires.ftc.teamcode.commands.elevator.SetElevatorPowerForTime;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeFromStack;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Config
@Autonomous
public class WBlue extends BaseOpMode {

    public static double xOffset = 1;
    public static double yOffset = 0;

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

    private InstantCommand pivot;

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

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

        telemetry.addData("spike pos", proppos);
        telemetry.update();
    }

    @Override
    public void runOnce() {
        switch (proppos) {
            case (1):
                purplePose = AutoConstantsState.BlueW.SPIKE_1;
                offPurplePose = AutoConstantsState.BlueW.SPIKE_1_BACK;
                stackAlignedPose = AutoConstantsState.BlueW.STACK_ALIGN;
                stackPose = AutoConstantsState.BlueW.STACK;
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
                stackAlignedPose = AutoConstantsState.BlueW.STACK_ALIGN;
                stackPose = AutoConstantsState.BlueW.STACK;
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
                stackAlignedPose = AutoConstantsState.BlueW.STACK_ALIGN;
                stackPose = AutoConstantsState.BlueW.STACK;
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
        stackAligned = new PIDToPoint(driveSS, stackAlignedPose, 1.25, 5);
        stack = new PIDToPoint(driveSS, stackPose, 1, 5);
        crossField = new PIDToPoint(driveSS, crossFieldPose, 5, 5);
        backdropAlign1 = new PIDToPoint(driveSS, backdropAlignPose1, 1, 5);
        backdropAlign2 = new PIDToPoint(driveSS, backdropAlignPose2, 1, 5);
        backdropAlign3 = new PIDToPoint(driveSS, backdropAlignPose3, 1, 5);
        backdrop1 = new PIDToPoint(driveSS, backdrop1Pose, 1, 5);
        backdrop2 = new PIDToPoint(driveSS, backdrop2Pose, 1, 5);
        backdrop3 = new PIDToPoint(driveSS, backdrop3Pose, 1, 5);
        park = new PIDToPoint(driveSS, parkPose, 1, 5);


        schedule(new SequentialCommandGroup(
                purple,
                offPurple,
                stackAligned,
                stack,

                //intake at stack
                new IntakeFromStack(intakeSS, 5),
                new WaitCommand(300),
                armPoisedGroup,
                new WaitCommand(200),
                grabbersClosed,


                //run away and eject
                new SetIntakePower(intakeSS, 1),
                crossField,
//                new SetIntakePower(intakeSS, 0),
//
//                new WaitCommand(250),
//                new AutoArmBack(armSS),
//
//                backdropAlign1,

                new ParallelCommandGroup(
                        backdropAlign1,
                        new SequentialCommandGroup(
                                new SetIntakePower(intakeSS, 0),
                                new WaitCommand(250),
                                new AutoArmBack(armSS),
                                new SetElevatorPowerForTime(eleSS, -1, 150)
                        )
                ),

                backdrop1.withTimeout(400),
                grabbersOpen,

                backdropAlign3,
                new WaitCommand(100),
                armIdleGroup,
                new LowerElevator(eleSS),

                new InstantCommand(() -> driveSS.setDwPose(new Pose2d(driveSS.getDwPose().x + xOffset, driveSS.getDwPose().y + yOffset, driveSS.getDwPose().theta))),
            //cycle 2
                crossField,
                stackAligned,
                stack,
                //intake at stack
                new IntakeFromStack(intakeSS, 4),
                new WaitCommand(400),
                armPoisedGroup,
                new WaitCommand(200),
                grabbersClosed,

                //run away and eject
                new SetIntakePower(intakeSS, 1),
                crossField,
//                new SetIntakePower(intakeSS, 0),
//                new WaitCommand(250),
//                new AutoArmBack(armSS),
//
//                backdropAlign2,

                new ParallelCommandGroup(
                        backdropAlign2,
                        new SequentialCommandGroup(
                                new SetIntakePower(intakeSS, 0),
                                new WaitCommand(250),
                                new AutoArmBack(armSS),
                                new SetElevatorPowerForTime(eleSS, -1, 300)
                        )
                ),

                backdrop2.withTimeout(400),
                grabbersOpen,

                backdropAlign3,
                new WaitCommand(100),
                armIdleGroup,
                new LowerElevator(eleSS),

                //ending
                park
        ));

//        schedule(
//                new SequentialCommandGroup(
//                        p2pTest1,
//                        p2pTest2
//                )
//        );
    }

    @Override
    public void run() {
        super.run();

        robot.read(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.loop(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.write(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.clearBulkCache();

//        driveSS.drive.xController.setPID(Constants.X_COEFFS.kP, Constants.X_COEFFS.kI, Constants.X_COEFFS.kD);
//        driveSS.drive.yController.setPID(Constants.Y_COEFFS.kP, Constants.Y_COEFFS.kI, Constants.Y_COEFFS.kD);
        driveSS.drive.thetaController.setCoefficients(Constants.THETA_COEFFS);

        tad("target", driveSS.getTargetPose());
        telemetry.addData("target pose", driveSS.getTargetPose());
        telemetry.addData("spike", proppos);
        telemetry.addData("current", driveSS.getDwPose());
        tal();
        tad("reachedd x", driveSS.drive.reachedXTarget(0.5, driveSS.getDwPose()));
        tad("reachedd y", driveSS.drive.reachedYTarget(0.5, driveSS.getDwPose()));
        tad("reachedd theta", driveSS.drive.reachedThetaTarget(2, driveSS.getDwPose()));

        tad("intake current", intakeSS.getAverageCurrent());

        tau();
    }
}
