package org.firstinspires.ftc.teamcode.opmodes.auto.state;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;

@Config
public class AutoConstantsState {

    public static double FL_STATIC = 0.007;
    public static double FR_STATIC = 0.0085;
    public static double RL_STATIC = 0.0087;
    public static double RR_STATIC = 0.0075;

    @Config
    public static class BlueBD {
        public static Pose2d START = new Pose2d(18.50, 63, Math.toRadians(-90));

        public static Pose2d SPIKE_1_PREP = new Pose2d(28.5, 40, Math.toRadians(-90));
        public static Pose2d SPIKE_1 = new Pose2d(28.5, 40, Math.toRadians(-90));
        public static Pose2d SPIKE_1_BACK = new Pose2d(28.5, 40, Math.toRadians(-90));

        public static Pose2d SPIKE_2_PREP = new Pose2d(18, 35, Math.toRadians(-90));
        public static Pose2d SPIKE_2 = new Pose2d(18, 35, Math.toRadians(-90));
        public static Pose2d SPIKE_2_BACK = new Pose2d(18, 35, Math.toRadians(-90));

        public static Pose2d SPIKE_3_PREP = new Pose2d(18, 50, Math.toRadians(-90));
        public static Pose2d SPIKE_3 = new Pose2d(13, 32, Math.toRadians(180));
        public static Pose2d SPIKE_3_BACK = new Pose2d(13, 32, Math.toRadians(180));

        public static Pose2d BD_1 = new Pose2d(56.5, 41, Math.toRadians(180));
        public static Pose2d BD_2 = new Pose2d(56.5, 36, Math.toRadians(180));
        public static Pose2d BD_3 = new Pose2d(56.5, 31, Math.toRadians(180));

        public static Pose2d BD_1_OFF = new Pose2d(BD_1.x - 5, BD_1.y + 0.5, BD_1.theta);
        public static Pose2d BD_2_OFF = new Pose2d(BD_2.x - 5, BD_2.y + 0.5, BD_2.theta);
        public static Pose2d BD_3_OFF = new Pose2d(BD_3.x - 5, BD_3.y + 0.5, BD_3.theta);

        public static Pose2d CROSS_FIELD_BD = new Pose2d(20, 60, Math.toRadians(180));
        public static Pose2d CROSS_FIELD_W = new Pose2d(-46, 60, Math.toRadians(180));

        public static Pose2d STACK_ALIGNED = new Pose2d(-54, 36, Math.toRadians(180));
        public static Pose2d STACK = new Pose2d(-58, 36, Math.toRadians(180));

        public static Pose2d PARK = new Pose2d(48, 56, Math.toRadians(180));
    }

    @Config
    public static class BlueW {
        public static Pose2d START = new Pose2d(-39.5, 63, Math.toRadians(-90));

        public static Pose2d SPIKE_1_PREP = new Pose2d(-39, 52, Math.toRadians(-90));
        public static Pose2d SPIKE_1 = new Pose2d(-34.5, 32, Math.toRadians(0));
        public static Pose2d SPIKE_1_BACK = new Pose2d(-45, 35, Math.toRadians(-90));

//        public static Pose2d SPIKE_1_PREP = new Pose2d(-39, 52, Math.toRadians(-90));
//        public static Pose2d SPIKE_1 = new Pose2d(-39, 52, Math.toRadians(-90));
//        public static Pose2d SPIKE_1_BACK = new Pose2d(-34.5, 32, Math.toRadians(0));

        public static Pose2d SPIKE_2_PREP = new Pose2d(-36, 33, Math.toRadians(-90));
        public static Pose2d SPIKE_2 = new Pose2d(-36, 33, Math.toRadians(-90));
        public static Pose2d SPIKE_2_BACK = new Pose2d(-50, 42, Math.toRadians(-90));

        public static Pose2d SPIKE_3_PREP = new Pose2d(-39, 38, Math.toRadians(-135));
        public static Pose2d SPIKE_3 = new Pose2d(-39, 38, Math.toRadians(-135));
        public static Pose2d SPIKE_3_BACK = new Pose2d(-34, 24, Math.toRadians(180));

        public static Pose2d STACK_ALIGN_1 = new Pose2d(-48, 10.5, Math.toRadians(180));
        public static Pose2d STACK_ALIGN_2 = new Pose2d(-48, 11.5, Math.toRadians(180));
        public static Pose2d STACK_ALIGN_3 = new Pose2d(-48, 10.5, Math.toRadians(180));

        public static Pose2d STACK_1 = new Pose2d(-58.5, 12, Math.toRadians(180));
        public static Pose2d STACK_2 = new Pose2d(-59.5, 13, Math.toRadians(180));
        public static Pose2d STACK_3 = new Pose2d(-58.5, 12, Math.toRadians(180));

        public static Pose2d INTERMEDIAN = new Pose2d(30, 10.5, Math.toRadians(180));

        public static Pose2d BD_1 = new Pose2d(50, 38.5, Math.toRadians(180));
        public static Pose2d BD_2 = new Pose2d(50, 31.5, Math.toRadians(180));
        public static Pose2d BD_3 = new Pose2d(50, 29, Math.toRadians(180));

        public static Pose2d BD_1_OFF = new Pose2d(BD_1.x - 4, BD_1.y + 0.5, BD_1.theta);
        public static Pose2d BD_2_OFF = new Pose2d(BD_2.x - 4, BD_2.y + 0.5, BD_2.theta);
        public static Pose2d BD_3_OFF = new Pose2d(BD_3.x - 4, BD_3.y + 0.5, BD_3.theta);

        public static Pose2d PARK = new Pose2d(48, 16, Math.toRadians(180));
    }

    @Config
    public static class BlueWWall {
        public static Pose2d START = new Pose2d(-39.5, 63, Math.toRadians(-90));

        public static Pose2d SPIKE_1_PREP = new Pose2d(-39, 52, Math.toRadians(-90));
        public static Pose2d SPIKE_1 = new Pose2d(-34.5, 32, Math.toRadians(0));
        public static Pose2d SPIKE_1_BACK = new Pose2d(-45, 35, Math.toRadians(-90));

        public static Pose2d SPIKE_2_PREP = new Pose2d(-36, 34, Math.toRadians(-90));
        public static Pose2d SPIKE_2 = new Pose2d(-36, 34, Math.toRadians(-90));
        public static Pose2d SPIKE_2_BACK = new Pose2d(-50, 42, Math.toRadians(-90));

        public static Pose2d SPIKE_3_PREP = new Pose2d(-39, 38, Math.toRadians(-135));
        public static Pose2d SPIKE_3 = new Pose2d(-39, 38, Math.toRadians(-135));
        public static Pose2d SPIKE_3_BACK = new Pose2d(-34, 24, Math.toRadians(180));

        public static Pose2d BD_1 = new Pose2d(54, 41, Math.toRadians(180));
        public static Pose2d BD_2 = new Pose2d(54, 36, Math.toRadians(180));
        public static Pose2d BD_3 = new Pose2d(54, 31, Math.toRadians(180));

        public static Pose2d BD_1_OFF = new Pose2d(BD_1.x - 5, BD_1.y + 0.5, BD_1.theta);
        public static Pose2d BD_2_OFF = new Pose2d(BD_2.x - 5, BD_2.y + 0.5, BD_2.theta);
        public static Pose2d BD_3_OFF = new Pose2d(BD_3.x - 5, BD_3.y + 0.5, BD_3.theta);

        public static Pose2d CROSS_FIELD_BD = new Pose2d(20, 59, Math.toRadians(180));
        public static Pose2d CROSS_FIELD_W = new Pose2d(-46, 59, Math.toRadians(180));

        public static Pose2d STACK_ALIGNED = new Pose2d(-54, 36, Math.toRadians(180));
        public static Pose2d STACK = new Pose2d(-59, 36, Math.toRadians(180));

        public static Pose2d PARK = new Pose2d(48, 56, Math.toRadians(180));
    }

    @Config
    public static class RedBD {
        public static Pose2d START = new Pose2d(18.50, -63, Math.toRadians(90));

        public static Pose2d SPIKE_3_PREP = new Pose2d(28.5, -40, Math.toRadians(90));
        public static Pose2d SPIKE_3 = new Pose2d(28.5, -40, Math.toRadians(90));
        public static Pose2d SPIKE_3_BACK = new Pose2d(28.5, -40, Math.toRadians(90));

        public static Pose2d SPIKE_2_PREP = new Pose2d(18, -33.5, Math.toRadians(90));
        public static Pose2d SPIKE_2 = new Pose2d(18, -33.5, Math.toRadians(90));
        public static Pose2d SPIKE_2_BACK = new Pose2d(18, -33.5, Math.toRadians(90));

        public static Pose2d SPIKE_1_PREP = new Pose2d(10, 34, Math.toRadians(90));
        public static Pose2d SPIKE_1 = new Pose2d(10, 34, Math.toRadians(180));
        public static Pose2d SPIKE_1_BACK = new Pose2d(10, 42, Math.toRadians(180));

        public static Pose2d BD_3 = new Pose2d(58, -39, Math.toRadians(180));
        public static Pose2d BD_2 = new Pose2d(58, -38, Math.toRadians(180));
        public static Pose2d BD_1 = new Pose2d(58, -30, Math.toRadians(180));

        public static Pose2d BD_3_OFF = new Pose2d(BD_3.x - 4, BD_3.y + 0.5, BD_3.theta);
        public static Pose2d BD_2_OFF = new Pose2d(BD_2.x - 4, BD_2.y + 0.5, BD_2.theta);
        public static Pose2d BD_1_OFF = new Pose2d(BD_1.x - 4, BD_1.y + 0.5, BD_1.theta);

        public static Pose2d CROSS_FIELD_BD = new Pose2d(20, -60, Math.toRadians(180));
        public static Pose2d CROSS_FIELD_W = new Pose2d(-46, -60, Math.toRadians(180));

        public static Pose2d STACK_ALIGNED = new Pose2d(-54, -36, Math.toRadians(180));
        public static Pose2d STACK = new Pose2d(-57.25, -34, Math.toRadians(180));

        public static Pose2d PARK = new Pose2d(48, -56, Math.toRadians(180));
    }

    @Config
    public static class RedW {
        public static Pose2d START = new Pose2d(-39.5, -63, Math.toRadians(90));

        public static Pose2d SPIKE_3_PREP = new Pose2d(-39, -52, Math.toRadians(90));
        public static Pose2d SPIKE_3 = new Pose2d(-34.5, -32, Math.toRadians(0));
        public static Pose2d SPIKE_3_BACK = new Pose2d(-45, -35, Math.toRadians(90));

        public static Pose2d SPIKE_2_PREP = new Pose2d(-36, -34, Math.toRadians(90));
        public static Pose2d SPIKE_2 = new Pose2d(-36, -34, Math.toRadians(90));
        public static Pose2d SPIKE_2_BACK = new Pose2d(-50, -42, Math.toRadians(90));

        public static Pose2d SPIKE_1_PREP = new Pose2d(-39, -36, Math.toRadians(145));
        public static Pose2d SPIKE_1 = new Pose2d(-39, -36, Math.toRadians(145));
        public static Pose2d SPIKE_1_BACK = new Pose2d(-34, -24, Math.toRadians(180));

        public static Pose2d STACK_ALIGN_1 = new Pose2d(-48, -10.5, Math.toRadians(180));
        public static Pose2d STACK_ALIGN_2 = new Pose2d(-48, -11.5, Math.toRadians(180));
        public static Pose2d STACK_ALIGN_3 = new Pose2d(-48, -10.5, Math.toRadians(180));

        public static Pose2d STACK_1 = new Pose2d(-58.5, -12, Math.toRadians(180));
        public static Pose2d STACK_2 = new Pose2d(-59.5, -13, Math.toRadians(180));
        public static Pose2d STACK_3 = new Pose2d(-58.5, -12, Math.toRadians(180));

        public static Pose2d INTERMEDIAN = new Pose2d(30, -10.5, Math.toRadians(180));

        public static Pose2d BD_3 = new Pose2d(54, -40, Math.toRadians(180));
        public static Pose2d BD_2 = new Pose2d(54, -38, Math.toRadians(180));
        public static Pose2d BD_1 = new Pose2d(54, -31.5, Math.toRadians(180));

        public static Pose2d BD_3_OFF = new Pose2d(BD_3.x - 4, BD_3.y + 0.5, BD_3.theta);
        public static Pose2d BD_2_OFF = new Pose2d(BD_2.x - 4, BD_2.y + 0.5, BD_2.theta);
        public static Pose2d BD_1_OFF = new Pose2d(BD_1.x - 4, BD_1.y + 0.5, BD_1.theta);

        public static Pose2d PARK = new Pose2d(48, -12, Math.toRadians(180));
    }
}
