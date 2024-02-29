package org.firstinspires.ftc.teamcode.opmodes.auto.state;

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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.drive.PIDToPoint;
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
public class BDBlue extends BaseOpMode {

    protected PIDToPoint p2pTest1;
    protected PIDToPoint p2pTest2;
    protected PIDToPoint p2pTest3;
    protected PIDToPoint p2pTest4;
    protected PIDToPoint p2pTest5;

    public static Pose2d targetPose1;
    public static Pose2d targetPose2;
    public static Pose2d targetPose3;
    public static Pose2d targetPose4;
    public static Pose2d targetPose5;

    private InstantCommand pivot;

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    public int proppos;

    public boolean pastX = false;

    @Override
    public void initialize() {
        RobotHardware.USING_TAGS = false;
        RobotHardware.USING_IMU = true;
        Constants.ELE_PID = false;
        auto = false;
        alliance = Alliance.BLUE_BD;
        super.initialize();

        propProcessor = new PropProcessor(Alliance.BLUE_BD);

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.C920)
                .addProcessor(propProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        schedule(new SequentialCommandGroup(
                armPoisedGroup,
                new WaitCommand(50),
                grabberLeftClose,
                grabberRightClose
        ));

        schedule(intakeDown);
    }

    @Override
    public void initLoop() {
        robot.read(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.loop(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.write(driveSS, intakeSS, eleSS, armSS, grabSS);

        CommandScheduler.getInstance().run();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING);

        proppos = propProcessor.getSpike();

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
                targetPose1 = AutoConstantsState.BlueBD.SPIKE_1;
                targetPose2 = AutoConstantsState.BlueBD.SPIKE_1_BACK;
                targetPose3 = AutoConstantsState.BlueBD.BD_1_OFF;
                targetPose4 = AutoConstantsState.BlueBD.BD_1;
                targetPose5 = AutoConstantsState.BlueBD.PARK;
                pivot = new InstantCommand(() -> armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.NORMAL);
                break;
            case (2):
                targetPose1 = AutoConstantsState.BlueBD.SPIKE_2;
                targetPose2 = AutoConstantsState.BlueBD.SPIKE_2_BACK;
                targetPose3 = AutoConstantsState.BlueBD.BD_2_OFF;
                targetPose4 = AutoConstantsState.BlueBD.BD_2;
                targetPose5 = AutoConstantsState.BlueBD.PARK;
                pivot = new InstantCommand(() -> armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.NORMAL);
                break;
            case (3):
                targetPose1 = AutoConstantsState.BlueBD.SPIKE_3;
                targetPose2 = AutoConstantsState.BlueBD.SPIKE_3_BACK;
                targetPose3 = AutoConstantsState.BlueBD.BD_3_OFF;
                targetPose4 = AutoConstantsState.BlueBD.BD_3;
                targetPose5 = AutoConstantsState.BlueBD.PARK;
                pivot = new InstantCommand(() -> armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.NORMAL);
                break;
        }



        p2pTest1 = new PIDToPoint(driveSS, targetPose1, 1.25, 5);
        p2pTest2 = new PIDToPoint(driveSS, targetPose2, 3, 5);
        p2pTest3 = new PIDToPoint(driveSS, targetPose3, 1.25, 5);
        p2pTest4 = new PIDToPoint(driveSS, targetPose4, 2, 5);
        p2pTest5 = new PIDToPoint(driveSS, targetPose5, 2, 3);

        schedule(new SequentialCommandGroup(
                p2pTest1,
                p2pTest2,
                autoArmBack,
                new WaitCommand(100),
                pivot,
                p2pTest3,
                p2pTest4,
                new WaitCommand(50),
                grabbersOpen,
                new WaitCommand(150),
                new ParallelCommandGroup(
                        p2pTest5,
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                armIdleGroup
                        )
                )
        ));
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
        telemetry.addData("current pose", driveSS.getDwPose());
        tal();
        tad("reachedd x", driveSS.drive.reachedXTarget(0.5, driveSS.getDwPose()));
        tad("reachedd y", driveSS.drive.reachedYTarget(0.5, driveSS.getDwPose()));
        tad("reachedd theta", driveSS.drive.reachedThetaTarget(2, driveSS.getDwPose()));

        tau();

        TelemetryPacket packet = new TelemetryPacket();

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getDwPose());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getTargetPose());

        dashboard.sendTelemetryPacket(packet);
    }
}
