package org.firstinspires.ftc.teamcode.opmodes.auto.state.nodes;

import android.util.Size;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.opmodes.auto.state.LogicNode;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class BDlueNodes extends BaseOpMode {

    public static Pose2d START = new Pose2d(0, 0, 0);

    public static Pose2d SPIKE_1 = new Pose2d(22, 14, 0);
    public static Pose2d SPIKE_1_BACK = new Pose2d(SPIKE_1.x - 3, 14, 0);

    public static Pose2d SPIKE_2 = new Pose2d(28.5, 8, 0);
    public static Pose2d SPIKE_2_BACK = new Pose2d(SPIKE_2.x - 3, 8, 0);

    public static Pose2d SPIKE_3 = new Pose2d(32, -2.5, Math.toRadians(-70));
    public static Pose2d SPIKE_3_BACK = new Pose2d(SPIKE_3.x - 3, 2, Math.toRadians(-90));

    public static Pose2d BD_1 = new Pose2d(21.5, 36, Math.toRadians(-90));
    public static Pose2d BD_1_OFF = new Pose2d(BD_1.x, BD_1.y - 6, BD_1.theta);
    public static Pose2d BD_2 = new Pose2d(30, 36, Math.toRadians(-90));
    public static Pose2d BD_2_OFF = new Pose2d(BD_2.x, BD_2.y - 6, BD_2.theta);
    public static Pose2d BD_3 = new Pose2d(33.5, 36, Math.toRadians(-90));
    public static Pose2d BD_3_OFF = new Pose2d(BD_3.x, BD_3.y - 6, BD_3.theta);

    public static Pose2d PARK = new Pose2d(6, 28, Math.toRadians(-90));

    private PropProcessor propProcessor;

    private VisionPortal visionPortal;

    public ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    public double cycleTime = 10;

    public double returnTime = 6;

    public double returnTimeRight = 5.5;
    public double returnTimeCenter = 6.0;
    public double returnTimeLeft = 6.3;

    private int detectionCase = 1;

    public LogicNode currentNode = new LogicNode("Nothing");

    public final LogicNode start = new LogicNode("Start");

    private final LogicNode placePurple = new LogicNode("Placing purple");

    private final LogicNode backUpFromPurple = new LogicNode("Backing up from purple");

    private final LogicNode moveToBackdrop = new LogicNode("Moving to backdrop");

    private final LogicNode pushAgainstBackdrop = new LogicNode("Push against backdrop");

    private final LogicNode waitWhileScoring = new LogicNode("Wait while scoring");

    private final LogicNode alignToCross = new LogicNode("Align to cross");

    private final LogicNode alignToIntake = new LogicNode("Align to intake");

    private final LogicNode intaking = new LogicNode("Intaking");

    private final LogicNode reverseToLeave = new LogicNode("Reverse to leave");

    private final LogicNode crossField = new LogicNode("Cross field");

    private final LogicNode park = new LogicNode("Park");

    private final LogicNode end = new LogicNode("End");

    private int cycles = 0;

    public Pose2d offset = new Pose2d(0, 0, 0);

    public static double xIncrement = 1;
    public static double xDriftIncrement = 0;
    public static double yDriftIncrement = 0;

    private final Pose2d driftAfterLeft = new Pose2d(0, 0, 0);
    private final Pose2d driftAfterMiddle = new Pose2d(0, 0, 0);
    private final Pose2d driftAfterRight = new Pose2d(0, 0, 0);
    private Pose2d driftAfterPurple = new Pose2d(0, 0, 0);

    private int intakeTries = 0;

    public static double timeToScore = 0.85, timeToIntake = 1.3, timeToReverse = 0.85;
    public static double movementTimeOut = 0.15;

    private Pose2d purplePose = new Pose2d(0, 0, 0);
    private Pose2d backUpPose = new Pose2d(0, 0, 0);
    private Pose2d lineUpForBackdrop = new Pose2d(0, 0, 0);
    private Pose2d lineUpForIntakePose = new Pose2d(0, 0, 0);
    private Pose2d scoringPoseYellow = new Pose2d(0, 0, 0);
    private Pose2d scoringPoseWhites = new Pose2d(0, 0, 0);

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

        initNodes();

        schedule(new SequentialCommandGroup(
                armPoisedGroup,
                new WaitCommand(50),
                grabberLeftClose,
                grabberRightClose,
                intakeDown
        ));
    }

    @Override
    public void initLoop() {
        super.initLoop();
    }

    @Override
    public void runOnce() {
        super.runOnce();
    }

    @Override
    public void run() {
        super.run();
    }

    private void initCommands() {

    }

    private void initPoses() {

    }

    private void initNodes() {
        start.addCondition(
                () -> detectionCase == 1, () -> {
                    autoTimer.reset();
                    driveSS.setTargetPose(purplePose);
                    timer.reset();
                },
                placePurple
        );
        start.addCondition(
                () -> detectionCase == 2, () -> {
                    autoTimer.reset();
                    driveSS.setTargetPose(purplePose);
                    timer.reset();
                },
                placePurple
        );
        start.addCondition(
                () -> detectionCase == 3, () -> {
                    autoTimer.reset();
                    driveSS.setTargetPose(purplePose);
                    timer.reset();
                },
                placePurple
        );

        placePurple.addPositionCondition(driveSS, 1, backUpPose, backUpFromPurple);

        backUpFromPurple.addPositionCondition(driveSS, 2, lineUpForBackdrop, moveToBackdrop);

//        moveToBackdrop.addPositionCondition(driveSS, 2, pushAgainstBackdrop, pushAgainstBackdrop);
    }
}
