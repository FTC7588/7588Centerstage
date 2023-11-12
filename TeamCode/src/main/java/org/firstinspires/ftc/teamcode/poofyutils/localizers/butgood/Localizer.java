package org.firstinspires.ftc.teamcode.poofyutils.localizers.butgood;

import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;

public interface Localizer {

    Pose2d poseEstimate = null;

    Pose2d poseVelocity = null;

    void update();
}
