package org.firstinspires.ftc.teamcode.opmodes.auto.offseason;

import static org.firstinspires.ftc.teamcode.Constants.ARM_PIVOT_MID;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_SHOULDER;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_WRIST;
import static org.firstinspires.ftc.teamcode.Constants.INT_TWO;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetShoulderPosition;
import org.firstinspires.ftc.teamcode.commands.drive.PIDToPoint;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.opmodes.auto.state.AutoConstantsState;
import org.firstinspires.ftc.teamcode.poofyutils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.poofyutils.PoofyDashboardUtil;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.hardware.CameraConfig;
import org.firstinspires.ftc.teamcode.poofyutils.localizers.butgood.AprilTagLocalizer2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous
public class WBlueTagTest extends BaseOpMode {

    public static double defaultPosTol = 1;
    public static double defaultHeadingTol = 5;
    public static double slowSpeed = 0.75;

    protected PIDToPoint purple;
    protected PIDToPoint prePurple;
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

    public static Pose2d prePurplePose;
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

    private AprilTagLocalizer2d frontLocalizer;
    private AprilTagLocalizer2d backLocalizer;

    private ElapsedTime time;

    public int proppos;

    public boolean pastX = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        RobotHardware.USING_TAGS = true;
        RobotHardware.USING_IMU = true;
        RobotHardware.USING_SENSORS = true;
        Constants.ELE_PID = false;
        auto = false;
        alliance = Alliance.RED;
        super.initialize();

//        propProcessor = new PropProcessor(Alliance.BLUE_W);
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(robot.C920)
//                .addProcessor(propProcessor)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .enableLiveView(true)
//                .setAutoStopLiveView(false)
//                .build();

//        frontLocalizer = new AprilTagLocalizer2d(
//            new CameraConfig(
//                RobotHardware.getInstance().C920,
//                Constants.C920_POSE,
//                Constants.C920_INTRINSICS,
//                Constants.C920_EXPOSURE,
//                Constants.C920_GAIN,
//                new Size(640, 480),
//                VisionPortal.StreamFormat.MJPEG
//            )
//        );
//
//        backLocalizer = new AprilTagLocalizer2d(
//            new CameraConfig(
//                    RobotHardware.getInstance().C930,
//                    Constants.C930_POSE,
//                    Constants.C930_INTRINSICS,
//                    Constants.C930_EXPOSURE,
//                    Constants.C930_GAIN,
//                    new Size(640, 480),
//                    VisionPortal.StreamFormat.MJPEG
//            )
//        );

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
        new SetIntakeAngle(intakeSS, INT_TWO).schedule();

        robot.read(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.loop(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.write(driveSS, intakeSS, eleSS, armSS, grabSS);

        CommandScheduler.getInstance().run();
    }

    @Override
    public void initLoop() {
        robot.read(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.loop(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.write(driveSS, intakeSS, eleSS, armSS, grabSS);

        CommandScheduler.getInstance().run();

        tal("=== FRONT VISION INFO ===");
        tad("Tag Pose", driveSS.getFrontLocalizer().getTagPose());
        tad("Tag Readings", driveSS.getFrontLocalizer().getTagReadings());
        tad("Camera Pose", driveSS.getFrontLocalizer().getCameraPose());
        tad("Robot Pose", driveSS.getFrontLocalizer().getPoseEstimate());
        tau();

//        frontLocalizer.update();
//        backLocalizer.update();
    }

    public void run() {


        robot.read(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.loop(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.write(driveSS, intakeSS, eleSS, armSS, grabSS);

        robot.clearBulkCache();

        tal("=== FRONT VISION INFO ===");
        tad("Tag Pose", driveSS.getFrontLocalizer().getTagPose());
        tad("Tag Readings", driveSS.getFrontLocalizer().getTagReadings());
        tad("Camera Pose", driveSS.getFrontLocalizer().getCameraPose());
        tad("Robot Pose", driveSS.getFrontLocalizer().getPoseEstimate());

        TelemetryPacket packet = new TelemetryPacket();

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getFrontLocalizer().getPoseEstimate());
        PoofyDashboardUtil.drawRobotPoseAlt(packet.fieldOverlay(), driveSS.getFrontLocalizer().getCameraPose());

        dashboard.sendTelemetryPacket(packet);

        tau();
    }

}
