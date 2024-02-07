package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.A;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.Y;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
@Config
public class DriveTesting extends BaseOpMode {

    protected PIDToPoint p2pTest1;
    protected PIDToPoint p2pTest2;
    protected PIDToPoint p2pTest3;
    protected PIDToPoint p2pTest4;

    public static Pose2d targetPose1 = new Pose2d(14, 14, 0);
    public static Pose2d targetPose2 = new Pose2d(10, 14, Math.toRadians(0));
    public static Pose2d targetPose3 = new Pose2d(22, 36, Math.toRadians(-90));
    public static Pose2d targetPose4 = new Pose2d(10, 25, Math.toRadians(-90));

//    protected double posTol = 0.25;
//    protected double headingTol = 1;

    @Override
    public void initialize() {
        RobotHardware.USING_IMU = true;
        Constants.ELE_PID = false;
        alliance = Alliance.BLUE;

        super.initialize();

        p2pTest1 = new PIDToPoint(driveSS, targetPose1, 0.5, 1);
        p2pTest2 = new PIDToPoint(driveSS, targetPose2, 2, 5);
        p2pTest3 = new PIDToPoint(driveSS, targetPose3, 0.5, 2);
        p2pTest4 = new PIDToPoint(driveSS, targetPose4, 0.5, 2);

        driveSS.drive.xController.updatePID(Constants.X_COEFFS.kP, Constants.X_COEFFS.kI, Constants.X_COEFFS.kD);
        driveSS.drive.yController.updatePID(Constants.Y_COEFFS.kP, Constants.Y_COEFFS.kI, Constants.Y_COEFFS.kD);
        driveSS.drive.thetaController.setCoefficients(Constants.THETA_COEFFS);


        gp1(A, 1).whenActive(new SequentialCommandGroup(
                p2pTest1,
                new WaitCommand(100),
                p2pTest2,
//                new WaitCommand(100),
                autoArmBack,
                p2pTest3,
                new WaitCommand(50),
                grabbersOpen,
                new WaitCommand(50),
                new ParallelCommandGroup(
                        p2pTest4,
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                armIdleGroup
                        )
                )
        ));

//        gp1(A, 1).whenActive(p2pTest1);
//        gp1(Y, 1).whenActive(p2pTest2);
//        gp1(X, 1).whenActive(p2pTest3);

        driveSS.setDefaultCommand(robotCentric);

        intakeDown.schedule();
        shoulderPosGrab.schedule();
        wristPosGrab.schedule();
        pivotPosMid.schedule();
    }

    @Override
    public void runOnce() {
        new SequentialCommandGroup(
                armPoisedGroup,
                new WaitCommand(50),
                grabbersClosed
        ).schedule();
    }

    @SuppressLint("DefaultLocale")
    public void run() {
        CommandScheduler.getInstance().run();

        super.run();

        driveSS.drive.xController.updatePID(Constants.X_COEFFS.kP, Constants.X_COEFFS.kI, Constants.X_COEFFS.kD);
        driveSS.drive.yController.updatePID(Constants.Y_COEFFS.kP, Constants.Y_COEFFS.kI, Constants.Y_COEFFS.kD);
        driveSS.drive.thetaController.setCoefficients(Constants.THETA_COEFFS);

        TelemetryPacket packet = new TelemetryPacket();

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getDwPose());
//        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getTagPose());

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("target pose", driveSS.getTargetPose());

        telemetry.addData("robot centric scheduled", robotCentric.isScheduled());
        telemetry.addData("PID to point scheduled", p2pTest1.isScheduled());
        telemetry.addData("PID to point finished", p2pTest1.isFinished());

        tau();

        robot.clearBulkCache();
    }
}
