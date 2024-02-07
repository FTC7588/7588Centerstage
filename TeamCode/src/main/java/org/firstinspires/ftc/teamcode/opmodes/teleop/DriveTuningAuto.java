package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.A;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.B;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.Y;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.drive.HoldTargetPosition;
import org.firstinspires.ftc.teamcode.commands.drive.PIDToPoint;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.poofyutils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.poofyutils.PoofyDashboardUtil;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;

@TeleOp
public class DriveTuningAuto extends BaseOpMode {

    protected PIDToPoint p2pTest1;
    protected PIDToPoint p2pTest2;
    protected PIDToPoint p2pTest3;
    protected PIDToPoint p2pTest4;
    protected PIDToPoint p2pTest5;

    protected SequentialCommandGroup p2pTests;

    protected HoldTargetPosition holdTarget;

    protected Pose2d targetPose1 = new Pose2d(0, 0, 0);
    protected Pose2d targetPose2 = new Pose2d(40, 0, Math.toRadians(0));
    protected Pose2d targetPose3 = new Pose2d(30, 30, Math.toRadians(-90));
    protected Pose2d targetPose4 = new Pose2d(10, 25, Math.toRadians(-90));
    protected Pose2d targetPose5 = new Pose2d(10, 26, Math.toRadians(-90));

    double posTol = 1;
    double headingTol = 3;

    @Override
    public void initialize() {
        RobotHardware.USING_IMU = true;
        Constants.ELE_PID = false;
        alliance = Alliance.BLUE;

        super.initialize();

        holdTarget = new HoldTargetPosition(driveSS);

        p2pTest1 = new PIDToPoint(driveSS, targetPose1, posTol, headingTol);
        p2pTest2 = new PIDToPoint(driveSS, targetPose2, posTol, headingTol);
        p2pTest3 = new PIDToPoint(driveSS, targetPose3, posTol, headingTol);
        p2pTest4 = new PIDToPoint(driveSS, targetPose4, posTol, headingTol);
        p2pTest5 = new PIDToPoint(driveSS, targetPose5, posTol, headingTol);

//        driveSS.drive.xController.setPID(Constants.X_COEFFS.kP, Constants.X_COEFFS.kI, Constants.X_COEFFS.kD);
//        driveSS.drive.yController.setPID(Constants.Y_COEFFS.kP, Constants.Y_COEFFS.kI, Constants.Y_COEFFS.kD);
        driveSS.drive.thetaController.setCoefficients(Constants.THETA_COEFFS);


//        gp1(A, 1).whenActive(new SequentialCommandGroup(
//                p2pTest1,
//                new WaitCommand(1000),
//                p2pTest2,
//                new WaitCommand(1000),
//                p2pTest3,
//                new WaitCommand(1000),
//                p2pTest4,
//                new WaitCommand(1000),
//                p2pTest5
//        ));

//        gp1(A, 1).whenActive(p2pTest1);
//        gp1(B, 1).whenActive(p2pTest2);
//        gp1(X, 1).whenActive(p2pTest3);
//        gp1(Y, 1).whenActive(p2pTest4);

//        p2pTests = new SequentialCommandGroup(p2pTest2, p2pTest1);

        gp1(A, 1).whenActive(p2pTest1);
        gp1(Y, 1).whenActive(p2pTest2);

        driveSS.setDefaultCommand(holdTarget);

        intakeDown.schedule();
        shoulderPosGrab.schedule();
        wristPosGrab.schedule();
        pivotPosMid.schedule();
    }

    @Override
    public void runOnce() {

    }

    @SuppressLint("DefaultLocale")
    public void run() {
        CommandScheduler.getInstance().run();

        super.run();

//        new SequentialCommandGroup(p2pTest1, p2pTest2).schedule();

//        driveSS.drive.xController.updatePID(Constants.X_COEFFS.kP, Constants.X_COEFFS.kI, Constants.X_COEFFS.kD);
//        driveSS.drive.yController.updatePID(Constants.Y_COEFFS.kP, Constants.Y_COEFFS.kI, Constants.Y_COEFFS.kD);
        driveSS.drive.thetaController.setCoefficients(Constants.THETA_COEFFS);

        TelemetryPacket packet = new TelemetryPacket();

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getCenterStageTagLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getDwPose());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getTargetPose());

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("current pose", driveSS.getDwPose());
        telemetry.addData("target pose", driveSS.getTargetPose());
        telemetry.addData("X error", driveSS.getXError());
        telemetry.addData("Y error", driveSS.getYError());
        telemetry.addData("Theta error", driveSS.getThetaError());

//        telemetry.addData("velo error", driveSS.drive.xController.getVelocityError());

        tal();
        tad("reachedd x", driveSS.drive.reachedXTarget(posTol, driveSS.getDwPose()));
        tad("reachedd y", driveSS.drive.reachedYTarget(posTol, driveSS.getDwPose()));
        tad("reachedd theta", driveSS.drive.reachedThetaTarget(headingTol, driveSS.getDwPose()));

//        telemetry.addData("robot centric scheduled", robotCentric.isScheduled());
//        telemetry.addData("PID to point scheduled", p2pTest1.isScheduled());
        telemetry.addData("PID to point finished", p2pTest1.isFinished());

        tau();

        robot.clearBulkCache();
    }
}
