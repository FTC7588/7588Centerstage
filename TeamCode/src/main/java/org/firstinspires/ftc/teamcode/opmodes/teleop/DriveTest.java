package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ResetIMU;
import org.firstinspires.ftc.teamcode.commands.arm.IncrementWristPosition;
import org.firstinspires.ftc.teamcode.commands.arm.SetEleArmPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetPivotPosition;
import org.firstinspires.ftc.teamcode.commands.arm.SetShoulderPosition;
import org.firstinspires.ftc.teamcode.commands.arm.SetShoulderTouch;
import org.firstinspires.ftc.teamcode.commands.arm.SetWristPosition;
import org.firstinspires.ftc.teamcode.commands.drive.EnableHeadingLock;
import org.firstinspires.ftc.teamcode.commands.drive.FieldCentric;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTag;
import org.firstinspires.ftc.teamcode.commands.drive.RobotCentric;
import org.firstinspires.ftc.teamcode.commands.drive.SetHeadingLock;
import org.firstinspires.ftc.teamcode.commands.drive.SetMaxSpeed;
import org.firstinspires.ftc.teamcode.commands.elevator.IncrementElevatorTarget;
import org.firstinspires.ftc.teamcode.commands.elevator.SetElevatorPower;
import org.firstinspires.ftc.teamcode.commands.elevator.SetElevatorTarget;
import org.firstinspires.ftc.teamcode.commands.grabber.SetGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.poofyutils.PoofyDashboardUtil;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;

import static org.firstinspires.ftc.teamcode.Constants.*;


@TeleOp
@Config
public class DriveTest extends BaseOpMode {


    private RobotCentric robotCentric;
    private FieldCentric fieldCentric;
    private FollowTag followTag;

    private ResetIMU resetIMU;

    protected SetMaxSpeed lowSpeed;
    protected SetMaxSpeed highSpeed;

    protected EnableHeadingLock enabledHeadingLock;
    protected EnableHeadingLock disableHeadingLock;

    protected SetHeadingLock forwardLock;
    protected SetHeadingLock leftLock;
    protected SetHeadingLock backLock;
    protected SetHeadingLock rightLock;

    private SetIntakePower intakeIn;
    private SetIntakePower intakeIdle;
    private SetIntakePower intakeOut;

    private SetIntakeAngle intakeUp;
    private SetIntakeAngle intakeDown;

    private SetElevatorPower eleUp;
    private SetElevatorPower eleIdle;
    private SetElevatorPower eleDown;

    private SetElevatorTarget eleTargetUp;
    private SetElevatorTarget eleTargetMid;
    private SetElevatorTarget eleTargetDown;

    private IncrementElevatorTarget eleIncUp;
    private IncrementElevatorTarget eleIncDown;

    private SetShoulderPosition shoulderPosDeposit;
    private SetShoulderPosition shoulderPosPoised;
    private SetShoulderPosition shoulderPosGrab;
    private SetShoulderPosition shoulderPosIdle;

    private SetWristPosition wristPosDown;
    private SetWristPosition wristPosPoised;
    private SetWristPosition wristPosGrab;
    private SetWristPosition wristPosUp;
    private IncrementWristPosition wristIncDown;
    private IncrementWristPosition wristIncUp;

    private SetPivotPosition pivotPosDown;
    private SetPivotPosition pivotPosUp;

    private SetGrabberPosition grabberClosed;
    private SetGrabberPosition grabberOpen;

    private SetShoulderTouch armTouchPad;

    protected SetEleArmPositions armDeposit;
    protected SetEleArmPositions armIdle;
    protected SetEleArmPositions armPoised;
    protected SetEleArmPositions armGrab;

    protected FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        super.initialize();

        //drive
        robotCentric = new RobotCentric(
                driveSS,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()
        );

        fieldCentric = new FieldCentric(
                driveSS,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()
        );

        followTag = new FollowTag(driveSS, new Pose2d(0, 0, Math.toRadians(0)));

        //armTouchPad = new SetShoulderTouch(armSS, () -> driver.gamepad.touchpad_finger_1_x);

        resetIMU = new ResetIMU(robot);

        lowSpeed = new SetMaxSpeed(driveSS, LOW_SPEED);
        highSpeed = new SetMaxSpeed(driveSS, HIGH_SPEED);

        enabledHeadingLock = new EnableHeadingLock(driveSS, true);
        disableHeadingLock = new EnableHeadingLock(driveSS, false);

        forwardLock = new SetHeadingLock(driveSS, 90);
        leftLock = new SetHeadingLock(driveSS, 180);
        backLock = new SetHeadingLock(driveSS, -90);
        rightLock = new SetHeadingLock(driveSS, 0);

        //intake
        intakeIn = new SetIntakePower(intakeSS, INTAKE_POWER);
        intakeIdle = new SetIntakePower(intakeSS, 0);
        intakeOut = new SetIntakePower(intakeSS, -INTAKE_POWER);

        intakeUp = new SetIntakeAngle(intakeSS, INT_UP);
        intakeDown = new SetIntakeAngle(intakeSS, INT_DOWN);

        //elevator
        eleUp = new SetElevatorPower(eleSS, ELE_POWER);
        eleIdle = new SetElevatorPower(eleSS, 0);
        eleDown = new SetElevatorPower(eleSS, -ELE_POWER);

        eleTargetUp = new SetElevatorTarget(eleSS, ELE_UP);
        eleTargetMid = new SetElevatorTarget(eleSS, ELE_MID);
        eleTargetDown = new SetElevatorTarget(eleSS, ELE_DOWN);

        eleIncUp = new IncrementElevatorTarget(eleSS, ELE_INCREMENT);
        eleIncDown = new IncrementElevatorTarget(eleSS, -ELE_INCREMENT);

        //arm
        shoulderPosDeposit = new SetShoulderPosition(armSS, ARM_SHOULDER_DEPOSIT);
        shoulderPosPoised = new SetShoulderPosition(armSS, POISED_SHOULDER);
        shoulderPosGrab = new SetShoulderPosition(armSS, GRAB_SHOULDER);
        shoulderPosIdle = new SetShoulderPosition(armSS, ARM_SHOULDER_IDLE);

        wristPosDown = new SetWristPosition(armSS, ARM_WRIST_DEPOSIT);
        wristPosPoised = new SetWristPosition(armSS, POISED_WRIST);
        wristPosGrab = new SetWristPosition(armSS, GRAB_WRIST);
        wristPosUp = new SetWristPosition(armSS, ARM_WRIST_IDLE);
        wristIncDown = new IncrementWristPosition(armSS, 0.1);
        wristIncUp = new IncrementWristPosition(armSS, -0.1);

        pivotPosDown = new SetPivotPosition(armSS, ARM_PIVOT_DOWN);
        pivotPosUp = new SetPivotPosition(armSS, ARM_PIVOT_DOWN);

        //grabber
        grabberClosed = new SetGrabberPosition(grabSS, GRABBER_CLOSED);
        grabberOpen = new SetGrabberPosition(grabSS, GRABBER_OPEN);

        armDeposit = new SetEleArmPositions(
                eleSS,
                armSS,
                0,
                ARM_SHOULDER_DEPOSIT,
                ARM_WRIST_DEPOSIT,
                ARM_PIVOT_DOWN
        );

        armIdle = new SetEleArmPositions(
                eleSS,
                armSS,
                100,
                POISED_SHOULDER,
                0.5,
                ARM_PIVOT_DOWN
        );

        armPoised = new SetEleArmPositions(
                eleSS,
                armSS,
                POISED_ELE,
                POISED_SHOULDER,
                POISED_WRIST,
                ARM_PIVOT_DOWN
        );

        armGrab = new SetEleArmPositions(
                eleSS,
                armSS,
                GRAB_ELE,
                GRAB_SHOULDER,
                GRAB_WRIST,
                ARM_PIVOT_DOWN
        );

        //controls
        gp1(A, 2).whenActive(robot::resetIMU);
        gp1(X, 2).toggleWhenActive(robotCentric, fieldCentric);
        gp1(Y, 2).whenActive(resetIMU);

        gp1(LEFT_BUMPER, 1).whenActive(lowSpeed).whenInactive(highSpeed);

        gp1(B, 1).toggleWhenActive(intakeIn, intakeIdle);
        gp1(X, 1).toggleWhenActive(intakeOut, intakeIdle);

        gp1(A, 1).whenActive(intakeDown);
        gp1(Y, 1).whenActive(intakeUp);

//        gp1(DPAD_UP, 1).whenActive(eleUp).whenInactive(eleIdle);
//        gp1(DPAD_DOWN, 1).whenActive(eleDown).whenInactive(eleIdle);

        gp1(DPAD_UP, 1).whenActive(eleTargetUp);
        gp1(DPAD_LEFT, 1).whenActive(eleTargetMid);
        gp1(DPAD_DOWN, 1).whenActive(eleTargetDown);
        gp1(DPAD_RIGHT, 1).whenActive(followTag);

        gp1(DPAD_DOWN, 2).whenActive(shoulderPosGrab);
        gp1(DPAD_LEFT, 2).whenActive(shoulderPosPoised);
        gp1(DPAD_UP, 2).whenActive(shoulderPosDeposit);

        gp1(X, 3).whenActive(wristPosDown);
        gp1(B, 3).whenActive(wristPosUp);
        gp1(A, 3).whenActive(wristIncDown);
        gp1(Y, 3).whenActive(wristIncUp);

        gp1(LEFT_BUMPER, 2).whenActive(pivotPosDown);
        gp1(RIGHT_BUMPER, 2).whenActive(pivotPosUp);

        gp1(B, 2).toggleWhenActive(grabberClosed, grabberOpen);

//        gp1(Y, 3).whenActive(forwardLock);
//        gp1(X, 3).whenActive(leftLock);
//        gp1(A, 3).whenActive(backLock);
//        gp1(B, 3).whenActive(rightLock);

        gp1(LEFT_BUMPER, 3).whenActive(armPoised);
        gp1(RIGHT_BUMPER, 3).whenActive(armGrab);

        //debug controls
        gp1(DPAD_LEFT, 3).toggleWhenActive(() -> DEBUG_GENERAL = true, () -> DEBUG_GENERAL = false);
        gp1(DPAD_DOWN, 3).toggleWhenActive(() -> DEBUG_DRIVE = true, () -> DEBUG_DRIVE = false);
        gp1(DPAD_RIGHT, 3).toggleWhenActive(() -> DEBUG_ELEVATOR = true, () -> DEBUG_ELEVATOR = false);
        gp1(DPAD_UP, 3).toggleWhenActive(() -> DEBUG_VISION = true, () -> DEBUG_VISION = false);

        //p2 controls
        gp2(DPAD_UP, 1).whileActiveContinuous(eleIncUp);
        gp1(DPAD_LEFT, 1).whenActive(eleTargetUp);
        gp2(DPAD_DOWN, 1).whileActiveContinuous(eleIncDown);

        gp2(Y, 1).whenActive(armDeposit);
        gp2(X, 1).whenActive(armPoised);
        gp2(B, 1).whenActive(armIdle);
        gp2(A, 1).whenActive(armGrab);

        gp2(LEFT_BUMPER, 1).toggleWhenActive(grabberOpen, grabberClosed);


        robotCentric.schedule();
        intakeDown.schedule();
//        shoulderPosDown.schedule();
//        wristPosDown.schedule();
//        pivotPosDown.schedule();
//        armTouchPad.schedule();
        resetIMU.schedule();

    }

    public void run() {
        CommandScheduler.getInstance().run();

        super.run();

        robot.read(driveSS, intakeSS, eleSS, armSS);

        robot.loop(driveSS, intakeSS, eleSS, armSS);

        robot.write(driveSS, intakeSS, eleSS, armSS);

        telemetry.addData("tp1_x", gamepad1.touchpad_finger_1_x);

                TelemetryPacket packet = new TelemetryPacket();

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getTagLocalizer().getCameraPose());

        dashboard.sendTelemetryPacket(packet);

        telemetry.update();

        robot.clearBulkCache();
    }




}
