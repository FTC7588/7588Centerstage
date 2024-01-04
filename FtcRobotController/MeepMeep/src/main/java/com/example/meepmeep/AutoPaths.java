package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class AutoPaths {

    public static class Blue {
        public static double push = 3;

        //starts
        public static Pose2d BD_START = new Pose2d(14.75, 63, Math.toRadians(-90));
        public static Pose2d W_START = new Pose2d(-36, 63, Math.toRadians(-90));

        //spikes
        public static Pose2d BD_SPIKE_ONE = new Pose2d(30, 32, Math.toRadians(-135));
        public static Pose2d BD_SPIKE_TWO = new Pose2d(15, 33, Math.toRadians(-90));
        public static Pose2d BD_SPIKE_THREE = new Pose2d(8, 32, Math.toRadians(-135));

        public static Pose2d W_SPIKE_ONE = new Pose2d(-36, 25, Math.toRadians(180));
        public static Pose2d W_SPIKE_TWO = new Pose2d(-31, 15, Math.toRadians(-90));
        public static Pose2d W_SPIKE_THREE = new Pose2d(-40, 18, Math.toRadians(-45));

        //to backdrop from spikes
        public static Pose2d W_BD_ONE_A = new Pose2d(-36, 24.99, Math.toRadians(180));
        public static Pose2d W_BD_ONE_B = new Pose2d(20, 12, Math.toRadians(180));
        public static double W_BD_ONE_B_TANGENT = Math.toRadians(0);
        public static double W_BD_ONE_C_TANGENT = Math.toRadians(20);

        public static Pose2d W_BD_TWO_A = new Pose2d(-31, 12, Math.toRadians(180));
        public static Pose2d W_BD_TWO_B = new Pose2d(20, 12, Math.toRadians(180));
        public static double W_BD_TWO_C_TANGENT = Math.toRadians(20);

        public static Pose2d W_BD_THREE_A = new Pose2d(-32, 12, Math.toRadians(180));
        public static Pose2d W_BD_THREE_B = new Pose2d(20, 12, Math.toRadians(180));
        public static double W_BD_THREE_C_TANGENT = Math.toRadians(20);

        //stacks

        //off backdrop
        public static Pose2d BD_ONE_OFF = new Pose2d(49, 42, Math.toRadians(180));
        public static Pose2d BD_TWO_OFF = new Pose2d(49, 36, Math.toRadians(180));
        public static Pose2d BD_THREE_OFF = new Pose2d(49, 28, Math.toRadians(180));

        //pushed into backdrop
        public static Pose2d BD_ONE_PUSH = new Pose2d(BD_ONE_OFF.getX() + push, BD_ONE_OFF.getY(), BD_ONE_OFF.getHeading());
        public static Pose2d BD_TWO_PUSH = new Pose2d(BD_TWO_OFF.getX() + push, BD_TWO_OFF.getY(), BD_TWO_OFF.getHeading());
        public static Pose2d BD_THREE_PUSH = new Pose2d(BD_THREE_OFF.getX() + push, BD_THREE_OFF.getY(), BD_THREE_OFF.getHeading());

        //parks
        public static Pose2d PARK_CORNER = new Pose2d(44, 60, Math.toRadians(180));
        public static Pose2d PARK_CENTER = new Pose2d(44, 14, Math.toRadians(180));
    }

    enum Paths {
        BS_A,
        BS_B,
        BS_C,
        W_A,
        W_B,
        W_C,

        //#1 - 2+0 both sides target completion time 6-7 seconds
        BS_0, //column A
        W_0,

        //#2 - 2+1 on wing side target completion time 12 seconds
        W_1,

        //#3 - 2+3 on wing side and 2+2 on backstage side 17 seconds
        BS_2,
        W_3,

        //#4 - 2+5 on wing side and 2+4 on backstage side fit into the 30 seconds
        BS_4,
        W_5,

        //#5 - 2+6 on backstage side and 2+7 on wing side through B fit into the 30 seconds
        BS_6,
        W_7
    }

    public static Paths path = Paths.W_0;

    public static double VEL_MAX = 65;
    public static double ACCEL_MAX = 65;
    public static double TRACK_WIDTH = 12;
    public static double WIDTH = 14;
    public static double HEIGHT = 14;
    public static double WAIT_SPIKE = 0.25;
    public static double WAIT_BD = 0.5;

    public static Image field;

    static {
        try {
            field = ImageIO.read(new File("D:/mario/Documents/Robotics/CenterStageField.png"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static Pose2d RED_BS_START = new Pose2d(14.75, 63, Math.toRadians(-90));
    public static Pose2d RED_W_START = new Pose2d(-39, 61.9, Math.toRadians(-90));

    public static Pose2d RED_W_SPIKE_1 = new Pose2d(-34.9, 32, Math.toRadians(5));
    public static Pose2d RED_W_SPIKE_2 = new Pose2d(-34.9, 32, Math.toRadians(-90));
    public static Pose2d RED_W_SPIKE_3 = new Pose2d(-35.9, 25, Math.toRadians(-180));

    public static Pose2d RED_STACK = new Pose2d(-61, 36, Math.toRadians(0));

    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(920, 120);

        //BS_0
        RoadRunnerBotEntity BS_A_0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(220), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(WIDTH, HEIGHT)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Blue.BD_START)
                                .lineToLinearHeading(Blue.BD_SPIKE_ONE)
                                .waitSeconds(WAIT_SPIKE)
                                .lineToLinearHeading(Blue.BD_ONE_OFF)
                                .lineToLinearHeading(Blue.BD_ONE_PUSH)
                                .waitSeconds(WAIT_BD)
                                .lineToLinearHeading(Blue.PARK_CORNER)
                                .build());

        RoadRunnerBotEntity BS_B_0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(220), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(WIDTH, HEIGHT)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Blue.BD_START)
                                .lineToLinearHeading(Blue.BD_SPIKE_TWO)
                                .waitSeconds(WAIT_SPIKE)
                                .lineToLinearHeading(Blue.BD_TWO_OFF)
                                .lineToLinearHeading(Blue.BD_TWO_PUSH)
                                .waitSeconds(WAIT_BD)
                                .lineToLinearHeading(Blue.PARK_CORNER)
                                .build());

        RoadRunnerBotEntity BS_C_0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(220), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(WIDTH, HEIGHT)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Blue.BD_START)
                                .lineToLinearHeading(Blue.BD_SPIKE_THREE)
                                .waitSeconds(WAIT_SPIKE)
                                .lineToLinearHeading(Blue.BD_THREE_OFF)
                                .lineToLinearHeading(Blue.BD_THREE_PUSH)
                                .waitSeconds(WAIT_BD)
                                .lineToLinearHeading(Blue.PARK_CORNER)
                                .build());

        //W_0
        RoadRunnerBotEntity W_A_0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(220), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(WIDTH, HEIGHT)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Blue.W_START)
                                .lineToLinearHeading(Blue.W_SPIKE_ONE)
                                .waitSeconds(WAIT_SPIKE)
                                .lineToLinearHeading(Blue.W_BD_ONE_A)
                                .splineToLinearHeading(Blue.W_BD_ONE_B, Blue.W_BD_ONE_B_TANGENT)
                                .splineToLinearHeading(Blue.BD_ONE_OFF, Blue.W_BD_ONE_C_TANGENT)
                                .lineToLinearHeading(Blue.BD_ONE_PUSH)
                                .waitSeconds(WAIT_BD)
                                .lineToLinearHeading(Blue.PARK_CENTER)
                                .build());

        RoadRunnerBotEntity W_B_0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(220), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(WIDTH, HEIGHT)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Blue.W_START)
                                .lineToLinearHeading(Blue.W_SPIKE_TWO)
                                .waitSeconds(WAIT_SPIKE)
                                .lineToLinearHeading(Blue.W_BD_TWO_A)
                                .lineToLinearHeading(Blue.W_BD_TWO_B)
                                .splineToLinearHeading(Blue.BD_TWO_OFF, Blue.W_BD_TWO_C_TANGENT)
                                .lineToLinearHeading(Blue.BD_TWO_PUSH)
                                .waitSeconds(WAIT_BD)
                                .lineToLinearHeading(Blue.PARK_CENTER)
                                .build());

        RoadRunnerBotEntity W_C_0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(VEL_MAX, ACCEL_MAX, Math.toRadians(220), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(WIDTH, HEIGHT)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Blue.W_START)
                                .lineToLinearHeading(Blue.W_SPIKE_THREE)
                                .waitSeconds(WAIT_SPIKE)
                                .lineToLinearHeading(Blue.W_BD_THREE_A)
                                .lineToLinearHeading(Blue.W_BD_THREE_B)
                                .splineToLinearHeading(Blue.BD_THREE_OFF, Blue.W_BD_THREE_C_TANGENT)
                                .lineToLinearHeading(Blue.BD_THREE_PUSH)
                                .waitSeconds(WAIT_BD)
                                .lineToLinearHeading(Blue.PARK_CENTER)
                                .build());

        //BS_A
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

        //BS_B
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

        //W_A
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

        //W_B
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
                meepMeep.setBackground(field)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(BS_A_1)
                        .addEntity(BS_A_2)
                        .addEntity(BS_A_3)
                        .start();
                break;
            case BS_B:
                meepMeep.setBackground(field)
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
                meepMeep.setBackground(field)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(W_A_1)
                        .addEntity(W_A_2)
                        .addEntity(W_A_3)
                        .start();
                break;
            case W_B:
                meepMeep.setBackground(field)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(W_B_1)
                        .start();
                break;
            case W_C:
                break;

            case BS_0:
                meepMeep.setBackground(field)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(BS_A_0)
                        .addEntity(BS_B_0)
                        .addEntity(BS_C_0)
                        .start();
                break;

            case W_0:
                meepMeep.setBackground(field)
                        .setDarkMode(true)
                        .addEntity(W_A_0)
                        .addEntity(W_B_0)
                        .addEntity(W_C_0)
                        .start();
                break;

            case W_1:

        }
    }
}