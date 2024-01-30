package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.A;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.drive.PIDToPoint;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.poofyutils.PoofyDashboardUtil;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;

@TeleOp
public class DriveTesting extends BaseOpMode {

    protected PIDToPoint p2pTest1;
    protected PIDToPoint p2pTest2;
    protected PIDToPoint p2pTest3;

    protected Pose2d targetPose1 = new Pose2d(0, 0, 0);
    protected Pose2d targetPose2 = new Pose2d(10, 10, 0);
    protected Pose2d targetPose3 = new Pose2d(10, 20, 0);

    @Override
    public void initialize() {
        RobotHardware.USING_IMU = true;
        Constants.ELE_PID = false;
        alliance = Alliance.BLUE;

        super.initialize();

        p2pTest1 = new PIDToPoint(driveSS, targetPose1, 2, 2);
        p2pTest2 = new PIDToPoint(driveSS, targetPose2, 2, 2);
        p2pTest3 = new PIDToPoint(driveSS, targetPose3, 1, 2);


        gp1(A, 1).whenActive(new SequentialCommandGroup(p2pTest1, p2pTest2, p2pTest3));

        driveSS.setDefaultCommand(robotCentric);

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
        telemetry.addData("PID to point scheduled", p2pTest1.isScheduled());
        telemetry.addData("PID to point finished", p2pTest1.isFinished());

        tau();

        robot.clearBulkCache();
    }
}
