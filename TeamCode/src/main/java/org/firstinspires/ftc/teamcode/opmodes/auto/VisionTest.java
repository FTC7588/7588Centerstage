package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.Constants.C920_INTRINSICS;
import static org.firstinspires.ftc.teamcode.Constants.C920_POSE;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.poofyutils.enums.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.hardware.CameraConfig;
import org.firstinspires.ftc.teamcode.poofyutils.localizers.butgood.AprilTagLocalizer2d;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
@Disabled
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

//        PropProcessor propProcessor = new PropProcessor(Alliance.BLUE);

        AprilTagLocalizer2d tagLocalizer = new AprilTagLocalizer2d(new CameraConfig(
                RobotHardware.getInstance().C920,
                C920_POSE,
                C920_INTRINSICS,
                12,
                255,
                new Size(640, 480),
                VisionPortal.StreamFormat.MJPEG
        ));

        telemetry.addLine("Waiting:");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

//            telemetry.addData("Spike Location", propProcessor.getSpike());

            telemetry.addLine("running");

            telemetry.update();

        }

    }

}
