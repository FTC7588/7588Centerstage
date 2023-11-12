package org.firstinspires.ftc.teamcode.poofyutils.geometry;

import kotlin.jvm.JvmField;

public class Pose2dSpline {


    @JvmField private final Pose2d pose;
    @JvmField private final double radians;

    public Pose2dSpline(Pose2d pose, double radians) {
        this.pose = pose;
        this.radians = radians;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getRadians() {
        return radians;
    }
}
