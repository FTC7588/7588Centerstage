package org.firstinspires.ftc.teamcode.poofyutils.localizers.butgood;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.poofyutils.hardware.CameraConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class AprilTagStreamer {

    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    private ArrayList<AprilTagDetection> tags;

    private Pose3d camToRobot;

    public AprilTagStreamer(CameraConfig config, AprilTagLibrary library) {
        this.camToRobot = config.getCamPose();

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(
                        config.getIntrinsics().getFx(),
                        config.getIntrinsics().getFy(),
                        config.getIntrinsics().getCx(),
                        config.getIntrinsics().getCy()
                )
                .setTagLibrary(library)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(config.getCamera())
                .addProcessor(tagProcessor)
                .setCameraResolution(config.getSize())
                .setStreamFormat(config.getFormat())
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gain = visionPortal.getCameraControl(GainControl.class);

        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure((long) config.getExposure(), TimeUnit.MILLISECONDS);

        gain.setGain(config.getGain());

        tags = tagProcessor.getDetections();
    }

    public void update() {
        tags = tagProcessor.getDetections();
    }

    public ArrayList<AprilTagDetection> getTags() {
        return tags;
    }

    public Pose3d getCamToRobot() {
        return camToRobot;
    }

    public void setStreamerEnabled(boolean enabled) {
        visionPortal.setProcessorEnabled(tagProcessor, enabled);
    }
}
