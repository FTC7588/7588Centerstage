package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.Constants.DEBUG_ARM;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_DRIVE;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_ELEVATOR;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_GENERAL;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_GRABBER;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_VISION;
import static org.firstinspires.ftc.teamcode.Constants.GRABBER_TWO_CLOSED;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.A;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.B;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.DPAD_DOWN;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.DPAD_LEFT;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.DPAD_RIGHT;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.DPAD_UP;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.LEFT_BUMPER;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.TOUCHPAD_FINGER_1;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.Y;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.poofyutils.PoofyDashboardUtil;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;


@TeleOp
@Config
public class DriveFinal extends BaseOpMode {

    @Override
    public void initialize() {

        RobotHardware.USING_IMU = true;
        Constants.ELE_PID = false;
        RobotHardware.USING_SENSORS = false;
        alliance = Alliance.BLUE;

        super.initialize();

        //p1 layer 1 controls
        gp1(LEFT_BUMPER, 1).whenActive(lowSpeed).whenInactive(highSpeed);
        gp1(RIGHT_BUMPER, 1).toggleWhenActive(intakeOut, intakeIdle);

        gp1(A, 1).whenActive(enabledHeadingLock).whenActive(backLock).whenInactive(disableHeadingLock);
        gp1(B, 1).whenActive(enabledHeadingLock).whenActive(rightLock).whenInactive(disableHeadingLock);
        gp1(X, 1).whenActive(enabledHeadingLock).whenActive(leftLock).whenInactive(disableHeadingLock);
        gp1(Y, 1).whenActive(enabledHeadingLock).whenActive(forwardLock).whenInactive(disableHeadingLock);

        gp1(DPAD_UP, 1).whenActive(intakeUp);
        gp1(DPAD_LEFT, 1).toggleWhenActive(intakeIn, intakeIdle);
        gp1(DPAD_RIGHT, 1).whenActive(resetIMU);
        gp1(DPAD_DOWN, 1).whenActive(intakeDown);

        //p1 layer 2 controls
        gp1(DPAD_UP, 2).whenActive(shoulderPosIdle);
        gp1(DPAD_LEFT, 2).whenActive(shoulderPosDeposit);
        gp1(DPAD_RIGHT, 2).whenActive(shoulderPosPoised);
        gp1(DPAD_DOWN, 2).whenActive(shoulderPosGrab);

        gp1(A, 2).whenActive(wristIncUp);
        gp1(Y, 2).whenActive(wristIncDown);
        gp1(X, 2).toggleWhenActive(pivotPosDown, pivotPosUp);
        gp1(B, 2).toggleWhenActive(grabbersOpen, grabbersClosed);

        gp1(LEFT_BUMPER, 2).whenActive(armPoised);
        gp1(RIGHT_BUMPER, 2).whenActive(armGrab);

        //p1 layer 3 controls
        gp1(DPAD_LEFT, 3).toggleWhenActive(() -> DEBUG_GENERAL = true, () -> DEBUG_GENERAL = false);
        gp1(DPAD_DOWN, 3).toggleWhenActive(() -> DEBUG_DRIVE = true, () -> DEBUG_DRIVE = false);
        gp1(DPAD_RIGHT, 3).toggleWhenActive(() -> DEBUG_INTAKE = true, () -> DEBUG_INTAKE = false);
        gp1(DPAD_UP, 3).toggleWhenActive(() -> DEBUG_VISION = true, () -> DEBUG_VISION = false);
        gp1(X, 3).toggleWhenActive(() -> DEBUG_ELEVATOR = true, () -> DEBUG_ELEVATOR = false);
        gp1(A, 3).toggleWhenActive(() -> DEBUG_ARM = true, () -> DEBUG_ARM = false);
        gp1(B, 3).toggleWhenActive(() -> DEBUG_GRABBER = true, () -> DEBUG_GRABBER = true);

        gp1(LEFT_BUMPER, 3).whenActive(incrementIntakeUp);
        gp1(RIGHT_BUMPER, 3).whenActive(incrementIntakeDown);

        gp1(TOUCHPAD_FINGER_1, 1).whileActiveContinuous(variableIntakeAngle);

        //p2 controls
        gp2(DPAD_UP).whileActiveContinuous(eleUp).whenInactive(eleIdle);
        gp2(DPAD_LEFT).whenActive(eleTargetHang);
        gp2(DPAD_RIGHT).whenActive(armPoisedGroup);
        gp2(DPAD_DOWN).whileActiveContinuous(eleDown).whenInactive(eleIdle);

//        gp2(DPAD_UP, 2).whenActive(eleIncUp);
//        gp2(DPAD_DOWN, 2).whenActive(eleIncDown);
//        gp2(DPAD_LEFT, 2).whenActive(droneHold);

//        gp2(A, 1).whenActive(armIdleGroup);
        gp2(A).whenActive(armIdleGroup);
        gp2(Y).whenActive(armDepositGroup);

//        gp2(B, 1).whenActive(pivotPosDown).whenInactive(pivotPosMid);
//        gp2(X, 1).whenActive(pivotPosUp).whenInactive(pivotPosMid);
//        gp2(GamepadKeys.Trigger.RIGHT_TRIGGER).whenActive(pivotPosRightDiag).whenInactive(pivotPosMid);
//        gp2(GamepadKeys.Trigger.LEFT_TRIGGER).whenActive(pivotPosLeftDiag).whenInactive(pivotPosMid);

//        gp2(GamepadKeys.Trigger.LEFT_TRIGGER).whenActive(() -> armSS.pivotPositionState = ArmSubsystem.PivotPositionState.LEFT).whenInactive(() -> armSS.pivotPositionState = ArmSubsystem.PivotPositionState.MID);
//        gp2(GamepadKeys.Trigger.RIGHT_TRIGGER).whenActive(() -> armSS.pivotPositionState = ArmSubsystem.PivotPositionState.RIGHT).whenInactive(() -> armSS.pivotPositionState = ArmSubsystem.PivotPositionState.MID);

        gp2(GamepadKeys.Trigger.LEFT_TRIGGER).whenActive(() -> armSS.setPivotPositionState(ArmSubsystem.PivotPositionState.LEFT)).whenInactive(() -> armSS.setPivotPositionState(ArmSubsystem.PivotPositionState.MID));
        gp2(GamepadKeys.Trigger.RIGHT_TRIGGER).whenActive(() -> armSS.setPivotPositionState(ArmSubsystem.PivotPositionState.RIGHT)).whenInactive(() -> armSS.setPivotPositionState(ArmSubsystem.PivotPositionState.MID));


//        gp2(B, () -> armState == ArmState.EXTENDED).toggleWhenActive(() -> {
//            pivotState = PivotRotatedState.NORMAL;
//            pivotPosDownDefalt = pivotPosNormDown;
//            pivotPosLeftDefault = pivotPosNormLeft;
//            pivotPosRightDefault = pivotPosNormRight;
//            pivotPosNormDown.schedule();
//            }, () -> {
//            pivotState = PivotRotatedState.ROTATED;
//            pivotPosDownDefalt = pivotPosRotDown;
//            pivotPosLeftDefault = pivotPosRotLeft;
//            pivotPosRightDefault = pivotPosRotRight;
//            pivotPosRotDown.schedule();
//        });

        gp2(B).whenActive(() -> armSS.toggleRotated());



//        gp2(GamepadKeys.Trigger.LEFT_TRIGGER, () -> pivotState == PivotState.ROTATED).whenActive(pivotPosRotLeft).whenInactive(pivotPosDownDefalt);
//        gp2(GamepadKeys.Trigger.RIGHT_TRIGGER, () -> pivotState == PivotState.ROTATED).whenActive(pivotPosRotRight).whenInactive(pivotPosDownDefalt);

        gp2(LEFT_BUMPER).toggleWhenActive(grabberLeftOpen, grabberLeftClose);
        gp2(RIGHT_BUMPER).toggleWhenActive(grabberRightOpen, grabberRightClose);

//        gp1(LEFT_BUMPER, 3).whenActive(droneHold);
        gp1(RIGHT_BUMPER, 3).whenActive(droneRelease).whenInactive(droneHold);

        robotCentric.schedule();
        intakeDown.schedule();
        shoulderPosGrab.schedule();
        wristPosGrab.schedule();
        pivotPosMid.schedule();
    }

    @SuppressLint("DefaultLocale")
    public void run() {
        CommandScheduler.getInstance().run();

        super.run();

//        if (armSS.getShoulderPosition() == ARM_SHOULDER_DEPOSIT) {
//            armState = ArmState.EXTENDED;
//        }

//        if (intakeSS.getPower() != 0) {
//            grabbersOpen.schedule();
//        }

        if (grabSS.getLeftPos() == Constants.GRABBER_ONE_CLOSED && grabSS.getRightPos() == Constants.GRABBER_TWO_CLOSED) {
            gamepad1.rumble(1, 1, 20);
            gamepad2.rumble(1, 1, 20);
        } else if (grabSS.getLeftPos() == Constants.GRABBER_ONE_CLOSED) {
            gamepad1.rumble(1, 0, 20);
            gamepad2.rumble(1, 0, 20);
        } else if (grabSS.getRightPos() == GRABBER_TWO_CLOSED) {
            gamepad1.rumble(0, 1, 20);
            gamepad2.rumble(0, 1, 20);
        }

        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("Pivot pos State", armSS.pivotPositionState);
        telemetry.addData("Pivot rot State", armSS.pivotRotatedState);

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getDwPose());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getTagPose());

        dashboard.sendTelemetryPacket(packet);

        tau();

        robot.clearBulkCache();
    }

}
