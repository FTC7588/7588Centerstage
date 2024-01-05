package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoConstants {

    public static class Blue {
        public static double push = 3;

        //starts
        public static Pose2d BD_START = new Pose2d(14.75, 63, Math.toRadians(-90));
        public static Pose2d W_START = new Pose2d(-36, 63, Math.toRadians(-90));

        //spikes
        public static Pose2d BD_SPIKE_ONE = new Pose2d(31.5, 32, Math.toRadians(-135));
        public static Pose2d BD_SPIKE_TWO = new Pose2d(15, 33, Math.toRadians(-90));
        public static Pose2d BD_SPIKE_THREE = new Pose2d(8, 32, Math.toRadians(-135));

        public static Pose2d W_SPIKE_ONE = new Pose2d(-36.5, 31, Math.toRadians(180));
        public static Pose2d W_SPIKE_TWO = new Pose2d(-31, 15, Math.toRadians(-90));
        public static Pose2d W_SPIKE_THREE = new Pose2d(-40, 18, Math.toRadians(-45));

        //stack positions
        public static Pose2d STACK_C = new Pose2d(-59.5,11,Math.toRadians(180));

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

        //to backdrop from stacks
        public static Pose2d STACK_BD_1_A = new Pose2d(20,11, Math.toRadians(180));
        public static double STACK_BD_1_A_TANGENT = Math.toRadians(0);
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

}
