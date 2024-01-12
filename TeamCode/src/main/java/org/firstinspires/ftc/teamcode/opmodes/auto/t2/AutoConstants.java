package org.firstinspires.ftc.teamcode.opmodes.auto.t2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class AutoConstants {

    @Config
    public static class Blue {
        public static double push = 3;

        //starts
        public static Pose2d BD_START = new Pose2d(18.50, 63, Math.toRadians(-90));
        public static Pose2d W_START = new Pose2d(-39.5, 63, Math.toRadians(-90));

        //spikes
        public static Pose2d BD_SPIKE_ONE = new Pose2d(31.5, 32, Math.toRadians(-135));
        public static Pose2d BD_SPIKE_TWO = new Pose2d(18, 31.5, Math.toRadians(-90));
        public static Pose2d BD_SPIKE_THREE = new Pose2d(10, 34, Math.toRadians(-135));

        public static Pose2d W_SPIKE_ONE = new Pose2d(-38.25, 35, Math.toRadians(180));
        public static Pose2d W_SPIKE_TWO = new Pose2d(-39, 12, Math.toRadians(-90));
        public static Pose2d W_SPIKE_THREE = new Pose2d(-37.5, 18, Math.toRadians(-45));

        //stack positions
        public static Pose2d STACK_A = new Pose2d(-59, 36, Math.toRadians(180));
        public static Pose2d STACK_C = new Pose2d(-59,12,Math.toRadians(180));

        //to backdrop from spikes
        public static Pose2d W_BD_ONE_A = new Pose2d(-31, 12, Math.toRadians(180));
        public static Pose2d W_BD_ONE_B = new Pose2d(20, 12, Math.toRadians(180));

        public static Pose2d W_BD_TWO_A = new Pose2d(-31, 12, Math.toRadians(180));
        public static Pose2d W_BD_TWO_B = new Pose2d(20, 12, Math.toRadians(180));

        public static Pose2d W_BD_THREE_A = new Pose2d(-32, 12, Math.toRadians(180));
        public static Pose2d W_BD_THREE_B = new Pose2d(20, 12, Math.toRadians(180));

        //to backdrop from stacks
        public static Pose2d STACK_BD_1_A = new Pose2d(20,11, Math.toRadians(180));

        public static Pose2d STACK_BD_2_A = new Pose2d(20, 58, Math.toRadians(180));
        public static Pose2d STACK_BD_2_B = new Pose2d(-30, 58, Math.toRadians(180));

        //off backdrop
        public static Pose2d W_BD_ONE_OFF = new Pose2d(54, 43, Math.toRadians(180));
        public static Pose2d W_BD_TWO_OFF = new Pose2d(54, 36.75, Math.toRadians(180));
        public static Pose2d W_BD_THREE_OFF = new Pose2d(54, 31, Math.toRadians(180));

        public static Pose2d BD_BD_ONE_OFF = new Pose2d(55, 43, Math.toRadians(180));
        public static Pose2d BD_BD_TWO_OFF = new Pose2d(55, 34, Math.toRadians(180));
        public static Pose2d BD_BD_THREE_OFF = new Pose2d(55, 29.5, Math.toRadians(180));

        //parks
        public static Pose2d PARK_CORNER = new Pose2d(44, 60, Math.toRadians(180));
        public static Pose2d PARK_CENTER = new Pose2d(44, 14, Math.toRadians(180));
    }


    @Config
    public static class Red {
        public static double push = 3;

        //starts
        public static Pose2d BD_START = new Pose2d(18.50, -63, Math.toRadians(90));
        public static Pose2d W_START = new Pose2d(-39.5, -63, Math.toRadians(90));

        //spikes
        public static Pose2d BD_SPIKE_THREE = new Pose2d(31, -33.5, Math.toRadians(135));
        public static Pose2d BD_SPIKE_TWO = new Pose2d(21, -29, Math.toRadians(135));
        public static Pose2d BD_SPIKE_ONE = new Pose2d(10, -34, Math.toRadians(180));

        public static Pose2d W_SPIKE_THREE = new Pose2d(-37.25, -35, Math.toRadians(-180));
        public static Pose2d W_SPIKE_TWO = new Pose2d(-39, -12, Math.toRadians(90));
        public static Pose2d W_SPIKE_ONE = new Pose2d(-37, -16.5, Math.toRadians(45));

        //stack positions
        public static Pose2d STACK_A = new Pose2d(-57.5, -36, Math.toRadians(-180));

        public static Pose2d STACK_C_1 = new Pose2d(-60, -12, Math.toRadians(-180));
        public static Pose2d STACK_C_2 = new Pose2d(-59.75, -12, Math.toRadians(-180));
        public static Pose2d STACK_C_3 = new Pose2d(-58.25,-12,Math.toRadians(-180));

        //to backdrop from spikes
        public static Pose2d W_BD_ONE_A = new Pose2d(-31, -12, Math.toRadians(-180));
        public static Pose2d W_BD_ONE_B = new Pose2d(20, -12, Math.toRadians(-180));

        public static Pose2d W_BD_TWO_A = new Pose2d(-31, -12, Math.toRadians(-180));
        public static Pose2d W_BD_TWO_B = new Pose2d(20, -12, Math.toRadians(-180));

        public static Pose2d W_BD_THREE_A = new Pose2d(-32, -12, Math.toRadians(-180));
        public static Pose2d W_BD_THREE_B = new Pose2d(20, -12, Math.toRadians(-180));

        //to backdrop from stacks
        public static Pose2d STACK_BD_1_A = new Pose2d(20,-11, Math.toRadians(-180));

        public static Pose2d STACK_BD_2_A = new Pose2d(20, -58, Math.toRadians(-180));
        public static Pose2d STACK_BD_2_B = new Pose2d(-30, -58, Math.toRadians(-180));

        //off backdrop
        public static Pose2d W_BD_THREE_OFF = new Pose2d(53, -42.5, Math.toRadians(-180));
        public static Pose2d W_BD_TWO_OFF = new Pose2d(53, -35.5, Math.toRadians(-180));
        public static Pose2d W_BD_ONE_OFF = new Pose2d(53, -29, Math.toRadians(-180));

        public static Pose2d BD_BD_THREE_OFF = new Pose2d(55, -43, Math.toRadians(-180));
        public static Pose2d BD_BD_TWO_OFF = new Pose2d(55, -35, Math.toRadians(-180));
        public static Pose2d BD_BD_ONE_OFF = new Pose2d(55, -31, Math.toRadians(-180));

        //parks
        public static Pose2d PARK_CORNER = new Pose2d(44, -60, Math.toRadians(-180));
        public static Pose2d PARK_CENTER = new Pose2d(44, -14, Math.toRadians(-180));
    }

}
