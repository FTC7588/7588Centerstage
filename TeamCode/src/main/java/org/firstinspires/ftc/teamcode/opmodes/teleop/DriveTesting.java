package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.Constants.FOLLOW_POSE;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.A;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.drive.PIDToPoint;
import org.firstinspires.ftc.teamcode.commands.drive.RobotCentric;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.poofyutils.PoofyDashboardUtil;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;

@TeleOp
public class DriveTesting extends BaseOpMode {

    protected PIDToPoint p2pTest;

    protected Pose2d targetPose = new Pose2d(0, 0, 0);

    @Override
    public void initialize() {
        RobotHardware.USING_IMU = true;
        Constants.ELE_PID = false;
        alliance = Alliance.BLUE;

        super.initialize();

        p2pTest = new PIDToPoint(driveSS, targetPose, 1, 5);

        gp1(A, 1).whenActive(p2pTest.andThen(new RobotCentric(
                driveSS,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()
        )));

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

        TelemetryPacket packet = new TelemetryPacket();

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getDwPose());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getTagPose());

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("robot centric scheduled", robotCentric.isScheduled());
        telemetry.addData("PID to point scheduled", p2pTest.isScheduled());
        telemetry.addData("PID to point finished", p2pTest.isFinished());

        tau();

        robot.clearBulkCache();
    }
}
