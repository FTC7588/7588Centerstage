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
import static org.firstinspires.ftc.teamcode.Constants.ARM_PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.ARM_SHOULDER_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Constants.ARM_SHOULDER_IDLE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_WRIST_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Constants.ARM_WRIST_IDLE;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_ARM;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_DRIVE;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_ELEVATOR;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_GENERAL;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_GRABBER;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.DEBUG_VISION;
import static org.firstinspires.ftc.teamcode.Constants.ELE_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.ELE_HANG;
import static org.firstinspires.ftc.teamcode.Constants.ELE_INCREMENT;
import static org.firstinspires.ftc.teamcode.Constants.ELE_MID;
import static org.firstinspires.ftc.teamcode.Constants.ELE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.ELE_UP;
import static org.firstinspires.ftc.teamcode.Constants.FLOOR_ELE;
import static org.firstinspires.ftc.teamcode.Constants.FLOOR_SHOULDER;
import static org.firstinspires.ftc.teamcode.Constants.FLOOR_WRIST;
import static org.firstinspires.ftc.teamcode.Constants.FOLLOW_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GRABBER_CLOSED;
import static org.firstinspires.ftc.teamcode.Constants.GRABBER_OPEN;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_ELE;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_SHOULDER;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_WRIST;
import static org.firstinspires.ftc.teamcode.Constants.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.INT_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.INT_UP;
import static org.firstinspires.ftc.teamcode.Constants.LOW_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.POISED_ELE;
import static org.firstinspires.ftc.teamcode.Constants.POISED_SHOULDER;
import static org.firstinspires.ftc.teamcode.Constants.POISED_WRIST;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import org.firstinspires.ftc.teamcode.commands.grabber.SetLeftGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.grabber.SetRightGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.poofyutils.MathUtil;
import org.firstinspires.ftc.teamcode.poofyutils.PoofyDashboardUtil;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.EulerAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;


@TeleOp
@Config
public class DriveTournament extends BaseOpMode {

    @Override
    public void initialize() {
        super.initialize();

        //p1 layer 1 controls
        gp1(LEFT_BUMPER, 1).whenActive(lowSpeed).whenInactive(highSpeed);
        gp1(RIGHT_BUMPER, 1).toggleWhenActive(followTag, robotCentric);

        gp1(B, 1).toggleWhenActive(intakeIn, intakeIdle);
        gp1(X, 1).toggleWhenActive(intakeOut, intakeIdle);
        gp1(A, 1).whenActive(intakeDown);
        gp1(Y, 1).whenActive(intakeUp);

        gp1(DPAD_UP, 1).whileActiveContinuous(eleIncUp);
        gp1(DPAD_DOWN, 1).whileActiveContinuous(eleIncDown);

        //p1 layer 2 controls
        gp1(DPAD_UP, 2).whenActive(shoulderPosIdle);
        gp1(DPAD_LEFT, 2).whenActive(shoulderPosDeposit);
        gp1(DPAD_RIGHT, 2).whenActive(shoulderPosPoised);
        gp1(DPAD_DOWN, 2).whenActive(shoulderPosGrab);

        gp1(A, 2).whenActive(wristIncUp);
        gp1(Y, 2).whenActive(wristIncDown);
        gp1(X, 2).toggleWhenActive(pivotPosDown, pivotPosUp);
        gp1(B, 2).toggleWhenActive(grabberOpen, grabberClosed);

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

        //p2 controls
        gp2(DPAD_UP, 1).whileActiveContinuous(eleIncUp);
        gp2(DPAD_RIGHT, 1).whenActive(eleTargetHang);
        gp2(DPAD_DOWN, 1).whileActiveContinuous(eleIncDown);

        gp2(Y, 1).whenActive(armDeposit);
        gp2(X, 1).whenActive(armPoised);
        gp2(B, 1).whenActive(armIdle);
        gp2(A, 1).whenActive(armGrab);

        gp2(LEFT_BUMPER, 1).toggleWhenActive(grabberLeftOpen, grabberLeftClose);
        gp2(RIGHT_BUMPER, 1).toggleWhenActive(grabberRightOpen, grabberRightClose);

        robotCentric.schedule();
        intakeDown.schedule();
        armPoised.schedule();
    }

    @SuppressLint("DefaultLocale")
    public void run() {
        CommandScheduler.getInstance().run();

//        if (grabSS.getLeftPos() == GRABBER_CLOSED) {
//            gamepad2.rumble(50);
//        } else if (grabSS.getRightPos() == GRABBER_CLOSED) {
//            gamepad2.rumble(50);
//        }

        super.run();

        TelemetryPacket packet = new TelemetryPacket();

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getRobotPose());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), FOLLOW_POSE);

        dashboard.sendTelemetryPacket(packet);

        tele.update();

        robot.clearBulkCache();
    }

}
