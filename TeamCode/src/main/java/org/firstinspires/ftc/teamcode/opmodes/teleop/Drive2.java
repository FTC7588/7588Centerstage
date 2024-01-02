package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.Constants.DEBUG_ARM;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_DRIVE;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_ELEVATOR;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_GENERAL;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_GRABBER;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_VISION;
import static org.firstinspires.ftc.teamcode.Constants.FOLLOW_POSE;
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
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.poofyutils.PoofyDashboardUtil;
import org.firstinspires.ftc.teamcode.poofyutils.enums.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;


@TeleOp
@Config
public class Drive2 extends BaseOpMode {

    @Override
    public void initialize() {
        alliance = Alliance.BLUE;

        super.initialize();

        //p1 layer 1 controls
        gp1(LEFT_BUMPER, 1).whenActive(lowSpeed).whenInactive(highSpeed);
//        gp1(TOUCHPAD_FINGER_1, 1).whenActive(followBackdrop).whenInactive(robotCentric);

        gp1(B, 1).toggleWhenActive(intakeIn, intakeIdle);
        gp1(X, 1).toggleWhenActive(intakeOut, intakeIdle);
        gp1(A, 1).whenActive(intakeDown);
        gp1(Y, 1).whenActive(intakeUp);

        gp1(DPAD_UP, 1).whileActiveContinuous(eleUp).whenInactive(eleIdle);
        gp1(DPAD_DOWN, 1).whileActiveContinuous(eleDown).whenInactive(eleIdle);

        gp1(DPAD_LEFT, 1).whenActive(pivotPosMid);

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
        gp2(DPAD_UP, 1).whileActiveContinuous(eleUp).whenInactive(eleIdle);
//        gp2(DPAD_RIGHT, 1).whenActive(eleTargetHang);
        gp2(DPAD_DOWN, 1).whileActiveContinuous(eleDown).whenInactive(eleIdle);

//        gp2(Y, 1).whenActive(armDeposit);
        gp2(B, 1).whenActive(armIdle);
        gp2(Y, 1).whenActive(armDepositGroup);

        gp2(LEFT_BUMPER, 1).toggleWhenActive(grabberLeftOpen, grabberLeftClose);
        gp2(RIGHT_BUMPER, 1).toggleWhenActive(grabberRightOpen, grabberRightClose);

//        variableIntakeAngle.schedule();
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

        autoGrab.schedule();

        telemetry.addData("touch x", driver.getTouchX());
        telemetry.addData("front", ((DistanceSensor) robot.frontCS).getDistance(DistanceUnit.CM));
        telemetry.addData("back", ((DistanceSensor) robot.backCS).getDistance(DistanceUnit.CM));
        telemetry.addData("front and back", intakeSS.isLoaded());

        TelemetryPacket packet = new TelemetryPacket();

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getRobotPose());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), FOLLOW_POSE);

        PoofyDashboardUtil.drawPoint(packet.fieldOverlay(), new Pose2d(60, 60, 0));

        dashboard.sendTelemetryPacket(packet);

        tau();

        robot.clearBulkCache();
    }

}
