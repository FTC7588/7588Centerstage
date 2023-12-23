package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class AutoPaths {

    enum Paths {
        BS_A,
        BS_B,
        BS_C,
        W_A,
        W_B,
        W_C
    }

    public static Paths path = Paths.W_B;

    public static double VEL_MAX = 65;
    public static double ACCEL_MAX = 65;
    public static double TRACK_WIDTH = 12;

    public static Pose2d RED_BS_START = new Pose2d(14.75, 63, Math.toRadians(-90));
    public static Pose2d RED_W_START = new Pose2d(-39, 61.9, Math.toRadians(-90));

    public static Pose2d RED_BS_SPIKE_1;
    public static Pose2d RED_BS_SPIKE_2;
    public static Pose2d RED_BS_SPIKE_3;

    public static Pose2d RED_W_SPIKE_1 = new Pose2d(-34.9, 32, Math.toRadians(5));
    public static Pose2d RED_W_SPIKE_2 = new Pose2d(-34.9, 32, Math.toRadians(-90));
    public static Pose2d RED_W_SPIKE_3 = new Pose2d(-35.9, 25, Math.toRadians(-180));

    public static Pose2d RED_STACK = new Pose2d(-61, 36, Math.toRadians(0));

    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(800, 120);

        RoadRunnerBotEntity BS_A_1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(220), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(14,14)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(RED_BS_START)
                                //drive to spike
                                .lineToSplineHeading(new Pose2d(26, 34, Math.toRadians(-90)))
                                .waitSeconds(0.2)
                                //move to stack
                                .lineToSplineHeading(new Pose2d(15, 50, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(-180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .forward(4)
                                //move to backdrop
                                .lineToSplineHeading(new Pose2d(-40, 51, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 32), Math.toRadians(330))
                                .waitSeconds(0.4)
                                //move to stack
                                .lineToSplineHeading(new Pose2d(15, 52, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(-180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .forward(4)
                                //move to backdrop
                                .lineToSplineHeading(new Pose2d(-40, 51, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 38), Math.toRadians(330))
                                .waitSeconds(0.4)
                                //move to stack
                                .lineToSplineHeading(new Pose2d(15, 52, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(-180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .forward(4)
                                //move to backdrop
                                .lineToSplineHeading(new Pose2d(-40, 51, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 38), Math.toRadians(330))
                                .build());

        RoadRunnerBotEntity BS_A_2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(220), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(14,14)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(RED_BS_START)
                                .lineToSplineHeading(new Pose2d(15, 31, Math.toRadians(-90)))
                                .waitSeconds(0.2)
                                .lineToSplineHeading(new Pose2d(15, 51, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(-180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .forward(4)
                                .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 35), Math.toRadians(330))
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(15, 52, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(-180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .forward(4)
                                .lineToSplineHeading(new Pose2d(-40, 51, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 38), Math.toRadians(330))
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(15, 52, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(-180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .forward(4)
                                .lineToSplineHeading(new Pose2d(-40, 51, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 38), Math.toRadians(330))
                                .waitSeconds(0.4)
//                                .lineToSplineHeading(new Pose2d(-20, 38, Math.toRadians(0)))
//                                .splineToSplineHeading(new Pose2d(-61, 24, Math.toRadians(0)), Math.toRadians(-180))
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(-20, 34, Math.toRadians(0)))
//                                .splineToSplineHeading(new Pose2d(49, 36, Math.toRadians(0)), Math.toRadians(350))
//                                .waitSeconds(0.2)
                                .build());

        RoadRunnerBotEntity BS_A_3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(220), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(14,14)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(RED_BS_START)
                                .lineToSplineHeading(new Pose2d(8, 34, Math.toRadians(-115)))
                                .waitSeconds(0.2)
                                .lineToSplineHeading(new Pose2d(15, 50, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(-180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .forward(4)
                                .lineToSplineHeading(new Pose2d(-40, 51, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 38), Math.toRadians(330))
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(15, 52, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(-180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .forward(4)
                                .lineToSplineHeading(new Pose2d(-40, 51, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 38), Math.toRadians(330))
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(15, 51, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(-180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .forward(4)
                                .lineToSplineHeading(new Pose2d(-40, 52, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 38), Math.toRadians(330))
                                .waitSeconds(0.4)
//                                .lineToSplineHeading(new Pose2d(-20, 38, Math.toRadians(0)))
//                                .splineToSplineHeading(new Pose2d(-61, 24, Math.toRadians(0)), Math.toRadians(-180))
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(-20, 34, Math.toRadians(0)))
//                                .splineToSplineHeading(new Pose2d(49, 36, Math.toRadians(0)), Math.toRadians(350))
//                                .waitSeconds(0.2)
                                .build());

        RoadRunnerBotEntity BS_B_1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(180), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(14, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_BS_START)
                                .setTangent(Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(10, 32, Math.toRadians(-90)), Math.toRadians(-120))
                                .setTangent(-50)
                                .lineToSplineHeading(new Pose2d(51, 36, Math.toRadians(0)))
                                .lineTo(new Vector2d(-60, 35.5))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(51, 35.5))
                                .waitSeconds(0.15)
                                .lineTo(new Vector2d(-60, 35.5))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(51, 35.5))
                                .waitSeconds(0.15)
                                .lineTo(new Vector2d(-10, 35.5))
                                .splineToSplineHeading(new Pose2d(-60, 24, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(-27, 32))
                                .splineToConstantHeading(new Vector2d(51, 36), Math.toRadians(-360))
                                .waitSeconds(0.15)
                                .lineTo(new Vector2d(-10, 35.5))
                                .splineToSplineHeading(new Pose2d(-60, 24, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(-27, 32))
                                .splineToConstantHeading(new Vector2d(51, 36), Math.toRadians(-360))
                                .waitSeconds(0.15)
                                .build()
                );

        RoadRunnerBotEntity BS_B_2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(180), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(14, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_BS_START)
                                .setTangent(Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(5, 35, Math.toRadians(-115)), Math.toRadians(-120))
                                .setTangent(-50)
                                .lineToSplineHeading(new Pose2d(51, 38, Math.toRadians(0)))
                                .lineTo(new Vector2d(-60, 35.5))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(51, 35.5))
                                .waitSeconds(0.15)
                                .lineTo(new Vector2d(-60, 35.5))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(51, 35.5))
                                .waitSeconds(0.15)
                                .lineTo(new Vector2d(-10, 35.5))
                                .splineToSplineHeading(new Pose2d(-60, 24, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(-27, 32))
                                .splineToConstantHeading(new Vector2d(51, 36), Math.toRadians(-360))
                                .waitSeconds(0.15)
                                .lineTo(new Vector2d(-10, 35.5))
                                .splineToSplineHeading(new Pose2d(-60, 24, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(-27, 32))
                                .splineToConstantHeading(new Vector2d(51, 36), Math.toRadians(-360))
                                .waitSeconds(0.15)
                                .build()
                );

        RoadRunnerBotEntity BS_B_3 = new DefaultBotBuilder(meepMeep)
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(180), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(14, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_BS_START)
                                .setTangent(Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(17, 35, Math.toRadians(-75)), Math.toRadians(-120))
                                .setTangent(-50)
                                .lineToSplineHeading(new Pose2d(51, 38, Math.toRadians(0)))
                                .lineTo(new Vector2d(-60, 35.5))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(51, 35.5))
                                .waitSeconds(0.15)
                                .lineTo(new Vector2d(-60, 35.5))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(51, 35.5))
                                .waitSeconds(0.15)
                                .lineTo(new Vector2d(-10, 35.5))
                                .splineToSplineHeading(new Pose2d(-60, 24, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(-27, 33))
                                .splineToConstantHeading(new Vector2d(51, 36), Math.toRadians(-360))
                                .waitSeconds(0.15)
                                .lineTo(new Vector2d(-10, 35.5))
                                .splineToSplineHeading(new Pose2d(-60, 24, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(-27, 33))
                                .splineToConstantHeading(new Vector2d(51, 36), Math.toRadians(-360))
                                .waitSeconds(0.15)
                                .build()
                );

        RoadRunnerBotEntity W_A_1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(180), Math.toRadians(150), TRACK_WIDTH)
                .setDimensions(14,14)
                .followTrajectorySequence(drive ->

                                drive.trajectorySequenceBuilder(RED_W_START)
                                        .lineToSplineHeading(RED_W_SPIKE_1)
                                        .lineToLinearHeading(RED_STACK)
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(0)))
                                        .splineToConstantHeading(new Vector2d(49, 39), Math.toRadians(330))
                                        .waitSeconds(0.4)
                                        .lineToSplineHeading(new Pose2d(10, 55, Math.toRadians(0)))
                                        .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(-170))
                                        .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(0)))
                                        .splineToConstantHeading(new Vector2d(49, 39), Math.toRadians(330))
                                        .waitSeconds(0.4)
                                        .lineToSplineHeading(new Pose2d(10, 55, Math.toRadians(0)))
                                        .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(-170))
                                        .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(0)))
                                        .splineToConstantHeading(new Vector2d(49, 39), Math.toRadians(330))
                                        .build()
                );

        RoadRunnerBotEntity W_A_2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(180), Math.toRadians(150), TRACK_WIDTH)
                .setDimensions(14,14)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(RED_W_START)
                                .lineToSplineHeading(RED_W_SPIKE_2)
                                .lineToLinearHeading(RED_STACK)
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(-58, 36, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 36), Math.toRadians(330))
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(10, 55, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(-170))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(-58, 36, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 39), Math.toRadians(330))
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(10, 55, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(-170))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(-58, 36, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 39), Math.toRadians(330))
                                .waitSeconds(0.4)
                                .build());


        RoadRunnerBotEntity W_A_3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(180), Math.toRadians(150), TRACK_WIDTH)
                .setDimensions(14,14)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(RED_W_START)
                                .lineToSplineHeading(RED_W_SPIKE_3)
                                .lineToLinearHeading(new Pose2d(-34, 36, Math.toRadians(0)))
                                .lineToLinearHeading(RED_STACK)
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 32), Math.toRadians(330))
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(10, 55, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(-170))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(-58, 36, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 38), Math.toRadians(330))
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(10, 55, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(-170))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(0)), Math.toRadians(-180))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(-58, 36, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(49, 38), Math.toRadians(330))
                                .waitSeconds(0.4)
                                .build());

        RoadRunnerBotEntity W_B_1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(180), Math.toRadians(150), TRACK_WIDTH)
                .setDimensions(14, 14)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(RED_W_START)
                                .lineToSplineHeading(RED_W_SPIKE_1)
                                .lineToLinearHeading(RED_STACK)
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(40, 36, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(47, 40), Math.toRadians(0))
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(40, 39.25, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-61, 36), Math.toRadians(180))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(40, 36, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(47, 40), Math.toRadians(0))
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(40, 39.25, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-61, 36), Math.toRadians(180))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(40, 36, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(47, 40), Math.toRadians(0))
                                .waitSeconds(0.4)
                                .build());



        switch (path) {
            case BS_A:
                meepMeep.setBackground(ImageIO.read(new File("D:/mario/Documents/Robotics/CenterStageField.png")))
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(BS_A_1)
                        .addEntity(BS_A_2)
                        .addEntity(BS_A_3)
                        .start();
                break;
            case BS_B:
                meepMeep.setBackground(ImageIO.read(new File("D:/mario/Documents/Robotics/CenterStageField.png")))
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(BS_B_1)
                        .addEntity(BS_B_2)
                        .addEntity(BS_B_3)
                        .start();
                break;
            case BS_C:
                break;
            case W_A:
                meepMeep.setBackground(ImageIO.read(new File("D:/mario/Documents/Robotics/CenterStageField.png")))
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(W_A_1)
                        .addEntity(W_A_2)
                        .addEntity(W_A_3)
                        .start();
                break;
            case W_B:
                meepMeep.setBackground(ImageIO.read(new File("D:/mario/Documents/Robotics/CenterStageField.png")))
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(W_B_1)
                        .start();
                break;
            case W_C:
                break;
        }
    }
}