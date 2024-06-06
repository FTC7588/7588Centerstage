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
public class DriveScrimmage extends BaseOpMode {

    @Override
    public void initialize() {

        RobotHardware.USING_IMU = true;
        Constants.ELE_PID = false;
        RobotHardware.USING_SENSORS = false;
        alliance = Alliance.BLUE;

        super.initialize();

        //p1 layer 1 controls
        gp1(LEFT_BUMPER).whenActive(lowSpeed).whenInactive(highSpeed);
        gp1(RIGHT_BUMPER, 1).toggleWhenActive(intakeOut, intakeIdle);

        //auto grab
        gp1(X, 1).whenActive(armAutoGrab);

        //arm toggle
        gp1(B, 1).whenActive(toggleArmStates);

        //ele up
        gp1(Y, 1).whileActiveContinuous(eleUp).whenInactive(eleIdle);

        //ele down
        gp1(A, 1).whileActiveContinuous(eleDown).whenInactive(eleIdle);

        //toggle rotate
        gp1(Y, 2).whenActive(() -> armSS.toggleRotated());

        //lean left
        gp1(X, 2).whenActive(() -> armSS.setPivotPositionState(ArmSubsystem.PivotPositionState.LEFT)).whenInactive(() -> armSS.setPivotPositionState(ArmSubsystem.PivotPositionState.MID));

        //lean right
        gp1(B, 2).whenActive(() -> armSS.setPivotPositionState(ArmSubsystem.PivotPositionState.RIGHT)).whenInactive(() -> armSS.setPivotPositionState(ArmSubsystem.PivotPositionState.MID));

        //grab toggle
        gp1(A, 2).whenActive(toggleGrabbers);

        //intake up
        gp1(DPAD_UP, 1).whenActive(intakeUp);

        //intake down
        gp1(DPAD_DOWN, 1).whenActive(intakeDown);

        //outtake
        gp1(DPAD_LEFT, 1).toggleWhenActive(intakeIn, intakeIdle);

        //reset imu
        gp1(DPAD_RIGHT, 1).whenActive(resetIMU);

        //launch drone
        gp1(DPAD_RIGHT, 2).whenActive(droneRelease).whenInactive(droneHold);

        robotCentric.schedule();
        intakeDown.schedule();
        setArmIdle.schedule();
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

//        if (grabSS.getLeftPos() == Constants.GRABBER_ONE_CLOSED && grabSS.getRightPos() == Constants.GRABBER_TWO_CLOSED) {
//            gamepad1.rumble(1, 1, 20);
//            gamepad2.rumble(1, 1, 20);
//        } else if (grabSS.getLeftPos() == Constants.GRABBER_ONE_CLOSED) {
//            gamepad1.rumble(1, 0, 20);
//            gamepad2.rumble(1, 0, 20);
//        } else if (grabSS.getRightPos() == GRABBER_TWO_CLOSED) {
//            gamepad1.rumble(0, 1, 20);
//            gamepad2.rumble(0, 1, 20);
//        }

        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("Pivot pos State", armSS.pivotPositionState);
        telemetry.addData("Pivot rot State", armSS.pivotRotatedState);
        tad("Wrist State", armSS.getWristState());
        tad("Shoulder State", armSS.getShoulderState());
        tad("Arm State", armSS.getArmState());

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getDwPose());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getTagPose());

        dashboard.sendTelemetryPacket(packet);

        tau();

        robot.clearBulkCache();
    }

}

