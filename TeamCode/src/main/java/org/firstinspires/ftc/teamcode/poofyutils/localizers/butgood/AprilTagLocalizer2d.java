package org.firstinspires.ftc.teamcode.poofyutils.localizers.butgood;

import org.firstinspires.ftc.teamcode.poofyutils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.poofyutils.MathUtil;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Vector3d;
import org.firstinspires.ftc.teamcode.poofyutils.hardware.CameraConfig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

public class AprilTagLocalizer2d implements Localizer {

    protected ArrayList<AprilTagStreamer> streamers = new ArrayList<>();

    protected LinkedHashMap<AprilTagDetection, Pose3d> tagsWithCamPoses;

    protected ArrayList<AprilTagDetection> usedTags = new ArrayList<>();

    protected Pose2d robotPose = new Pose2d(0, 0, 0);

    protected Pose2d cameraPose = new Pose2d(0,0,0);

    protected Pose2d tagPose = new Pose2d(0,0,0);

    protected Transform2d camToTag = new Transform2d();

    protected Transform2d coordinateFix2d = new Transform2d(0, 0, Math.toRadians(90));
    protected Transform3d coordinateFix3d = new Transform3d(
            new Vector3d(0, 0, 0),
            new Rotation3d(0, 0, Math.toRadians(90))
    );

    protected boolean detected = false;

    public AprilTagLocalizer2d(CameraConfig... configs) {
        for (CameraConfig config : configs) {
            streamers.add(new AprilTagStreamer(config, AprilTagCustomDatabase.getCenterStageTagLibrary()));
        }
    }

    @Override
    public void update() {
        usedTags = new ArrayList<>();
        consolidateLists();
        if (!tagsWithCamPoses.isEmpty()) {
            robotPose = lowestDecisionMarginStrategy2d(tagsWithCamPoses);
            detected = true;
        } else {
            detected = false;
        }
    }


    public boolean isDetected() {
        return detected;
    }

    protected void consolidateLists() {

        tagsWithCamPoses = new LinkedHashMap<>();
        for (int i = 0; i < streamers.size(); i++) {
            streamers.get(i).update();
            for (int j = 0; j < streamers.get(i).getTags().size(); j++) {
                //tags.add(MathUtil.tagTransformCamToRobot(streamers.get(i).getTags().get(j), streamers.get(i).getCamToRobot()));
                tagsWithCamPoses.put(streamers.get(i).getTags().get(j), streamers.get(i).getCamToRobot());
            }
        }
    }

    //strategies
    protected Pose2d lowestDecisionMarginStrategy2d(LinkedHashMap<AprilTagDetection, Pose3d> detections) {

        double lowestMargin = -1;
        AprilTagDetection lowestMarginTag = null;
        Pose3d lowestMarginCamPose = null;

        for (Map.Entry<AprilTagDetection, Pose3d> entry : detections.entrySet()) {
            if (lowestMargin < entry.getKey().decisionMargin || lowestMargin == -1) {
                lowestMargin = entry.getKey().decisionMargin;
                lowestMarginTag = entry.getKey();
                lowestMarginCamPose = entry.getValue();
            }
        }

        assert lowestMarginTag != null;

        tagPose = new Pose2d(
                lowestMarginTag.metadata.fieldPosition.get(0),
                lowestMarginTag.metadata.fieldPosition.get(1),
                MathUtil.quaternionToEuler(lowestMarginTag.metadata.fieldOrientation).yaw + Math.toRadians(90)
        );

        camToTag = new Transform2d(
                lowestMarginTag.ftcPose.x,
                lowestMarginTag.ftcPose.y,
                Math.toRadians(lowestMarginTag.ftcPose.yaw - 90)
        );

        return getRobotToTagPose2dAxes(tagPose, camToTag, lowestMarginCamPose);
    }

    protected Pose2d averageTagsStrategy(LinkedHashMap<AprilTagDetection, Pose3d> detections) {

        double x = 0;
        double y = 0;
        double yaw = 0;

        double num = 1;

        for (Map.Entry<AprilTagDetection, Pose3d> entry : detections.entrySet()) {
//            Pose2d tagPose = new Pose2d(
//                    entry.getKey().metadata.fieldPosition.get(0),
//                    entry.getKey().metadata.fieldPosition.get(1),
//                    MathUtil.quaternionToEuler(entry.getKey().metadata.fieldOrientation).yaw + Math.toRadians(90)
//            );
//
//            Transform2d camToTag = new Transform2d(
//                    entry.getKey().ftcPose.x,
//                    entry.getKey().ftcPose.y,
//                    Math.toRadians(entry.getKey().ftcPose.yaw - 90)
//            );
//
//            Pose2d robotPose = getRobotToTagPose2dAxes(tagPose, camToTag, entry.getValue());
//
//            x += robotPose.getX();
//            y += robotPose.getY();
//            yaw += robotPose.getTheta();
//            num++;
        }

        return new Pose2d(x/num, y/num, yaw/num);

    }

    //convert camera pose to robot pose
    protected Pose2d getRobotToTagPose2dAxes(Pose2d tagPose, Transform2d camToTag, Pose3d robotToCam) {
        cameraPose = tagPose.transformBy(camToTag.inverse()).transformBy(coordinateFix2d);

        return cameraPose.transformBy(robotToCam.toTransform2d().inverse());
    }

    public Pose2d getPoseEstimate() {
        return robotPose;
    }

    public Pose2d getCameraPose() {
        return cameraPose;
    }

    public Pose2d getTagPose() {
        return tagPose;
    }

    public Pose2d getTagReadings() {
        return new Pose2d(camToTag.getVector(), camToTag.getRotation());
    }

    public ArrayList<AprilTagDetection> getUsedTags() {
        return usedTags;
    }
}
