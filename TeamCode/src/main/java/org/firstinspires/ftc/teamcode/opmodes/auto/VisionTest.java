package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.poofyutils.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Disabled
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        PropProcessor propProcessor = new PropProcessor(PropProcessor.Alliance.BLUE);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(propProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        telemetry.addLine("Waiting:");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            telemetry.addData("Spike Location", propProcessor.getSpike());

            telemetry.addLine("running");

            telemetry.update();

        }

    }

}
