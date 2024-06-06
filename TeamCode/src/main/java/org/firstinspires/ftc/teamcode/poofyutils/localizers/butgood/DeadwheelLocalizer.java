package org.firstinspires.ftc.teamcode.poofyutils.localizers.butgood;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.poofyutils.filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Vector2d;

public class DeadwheelLocalizer implements Localizer {

    protected Pose2d pose;
    public com.acmerobotics.roadrunner.localization.Localizer localizer;
    private Vector2d velocity = new Vector2d();

    private final LowPassFilter xVelocity = new LowPassFilter(0.8, 0);
    private final LowPassFilter yVelocity = new LowPassFilter(0.8, 0);

    public DeadwheelLocalizer(RobotHardware robot, Pose2d initialPose) {
        this.pose = initialPose;
        this.localizer = new RRLocalizer(robot);
        localizer.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(initialPose.getX(), initialPose.getY(), initialPose.getTheta()));
    }

    public DeadwheelLocalizer(RobotHardware robot) {
        this.pose = new Pose2d(0, 0, 0);
        this.localizer = new RRLocalizer(robot);
        localizer.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d());
    }

    public void setPose(Pose2d pose) {
        localizer.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(pose.getX(), pose.getY(), pose.getTheta()));
        this.pose = pose;
    }

    public Pose2d getPoseEstimate() {
        return pose;
    }

    public double getHeading() {
        return pose.getTheta();
    }

    public Vector2d getVelocity() {
        return velocity;
    }

    @Override
    public void update() {
        localizer.update();
        com.acmerobotics.roadrunner.geometry.Pose2d rrpose = localizer.getPoseEstimate();
        pose = new Pose2d(rrpose.getX(), rrpose.getY(), rrpose.getHeading());
        try {
            velocity = new Vector2d(xVelocity.getValue(localizer.getPoseVelocity().getX()), yVelocity.getValue(localizer.getPoseVelocity().getY()));
        } catch (Exception ignored) {

        }
    }
}
