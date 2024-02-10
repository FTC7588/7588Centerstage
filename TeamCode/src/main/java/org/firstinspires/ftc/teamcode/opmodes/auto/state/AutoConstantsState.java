package org.firstinspires.ftc.teamcode.opmodes.auto.state;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;

@Config
public class AutoConstantsState {

    public static double FL_STATIC = 0.007;
    public static double FR_STATIC = 0.0085;
    public static double RL_STATIC = 0.0087;
    public static double RR_STATIC = 0.0075;

    public static class BlueBD {
        public static Pose2d START = new Pose2d(0, 0, 0);

        public static Pose2d SPIKE_1 = new Pose2d(22, 14, 0);
        public static Pose2d SPIKE_1_BACK = new Pose2d(SPIKE_1.x - 3, 14, 0);

        public static Pose2d SPIKE_2 = new Pose2d(28.5, 8, 0);
        public static Pose2d SPIKE_2_BACK = new Pose2d(SPIKE_2.x - 3, 8, 0);

        public static Pose2d SPIKE_3 = new Pose2d(32, -2.5, Math.toRadians(-70));
        public static Pose2d SPIKE_3_BACK = new Pose2d(SPIKE_3.x - 3, 2, Math.toRadians(-90));

        public static Pose2d BD_1 = new Pose2d(21.5, 36, Math.toRadians(-90));
        public static Pose2d BD_1_OFF = new Pose2d(BD_1.x, BD_1.y - 6, BD_1.theta);
        public static Pose2d BD_2 = new Pose2d(30, 36, Math.toRadians(-90));
        public static Pose2d BD_2_OFF = new Pose2d(BD_2.x, BD_2.y - 6, BD_2.theta);
        public static Pose2d BD_3 = new Pose2d(33.5, 36, Math.toRadians(-90));
        public static Pose2d BD_3_OFF = new Pose2d(BD_3.x, BD_3.y - 6, BD_3.theta);

        public static Pose2d PARK = new Pose2d(6, 28, Math.toRadians(-90));
    }

    public static class RedBD {
        public static Pose2d START = new Pose2d(0, 0, 0);

        public static Pose2d SPIKE_1 = new Pose2d(29, 5, Math.toRadians(90));
        public static Pose2d SPIKE_1_BACK = new Pose2d(SPIKE_1.x - 0, 5, Math.toRadians(90));

        public static Pose2d SPIKE_2 = new Pose2d(29.5, 6, 0);
        public static Pose2d SPIKE_2_BACK = new Pose2d(SPIKE_2.x - 4, 6, 0);

        public static Pose2d SPIKE_3 = new Pose2d(22, -1, Math.toRadians(0));
        public static Pose2d SPIKE_3_BACK = new Pose2d(SPIKE_3.x - 3, SPIKE_3.y, Math.toRadians(0));

        public static Pose2d BD_1 = new Pose2d(34, -35, Math.toRadians(90));
        public static Pose2d BD_2 = new Pose2d(28.5, -35, Math.toRadians(90));
        public static Pose2d BD_3 = new Pose2d(20.5, -35, Math.toRadians(90));

        public static Pose2d PARK = new Pose2d(6, -28, Math.toRadians(90));
    }

    public static class RedW {
        public static Pose2d START = new Pose2d(0, 0, 0);

        public static Pose2d SPIKE_1 = new Pose2d(22, 14, 0);
        public static Pose2d SPIKE_1_BACK = new Pose2d(SPIKE_1.x - 6, 14, 0);

        public static Pose2d SPIKE_2 = new Pose2d(28.5, 8, 0);
        public static Pose2d SPIKE_2_BACK = new Pose2d(SPIKE_2.x - 6, 8, 0);

        public static Pose2d SPIKE_3 = new Pose2d(32, -2.5, Math.toRadians(-70));
        public static Pose2d SPIKE_3_BACK = new Pose2d(SPIKE_3.x - 6, 2, Math.toRadians(-90));

        public static Pose2d BD_1 = new Pose2d(21.5, 36, Math.toRadians(-90));
        public static Pose2d BD_2 = new Pose2d(30, 36, Math.toRadians(-90));
        public static Pose2d BD_3 = new Pose2d(33.5, 36, Math.toRadians(-90));

        public static Pose2d PARK = new Pose2d(6, 28, Math.toRadians(-90));
    }

    public static class BlueW {
        public static Pose2d START = new Pose2d(-39.5, 63, Math.toRadians(-90));

        public static Pose2d SPIKE_1 = new Pose2d(-39, 32, Math.toRadians(-30));
        public static Pose2d SPIKE_1_BACK = new Pose2d(-45, 35, Math.toRadians(-90));

        public static Pose2d SPIKE_2 = new Pose2d(-48, 25, Math.toRadians(-90));
        public static Pose2d SPIKE_2_BACK = new Pose2d(SPIKE_2.x - 6, SPIKE_2.y, SPIKE_2.theta);

        public static Pose2d SPIKE_3 = new Pose2d(-37.5, 18, Math.toRadians(180));
        public static Pose2d SPIKE_3_BACK = new Pose2d(SPIKE_3.x - 6, SPIKE_3.y, SPIKE_3.theta);

        public static Pose2d STACK_ALIGN = new Pose2d(-48, 12, Math.toRadians(180));

        public static Pose2d STACK = new Pose2d(-58.5, 12, Math.toRadians(180));

        public static Pose2d INTERMEDIAN = new Pose2d(30, 12, Math.toRadians(180));

        public static Pose2d BD_1 = new Pose2d(53, 38, Math.toRadians(180));
        public static Pose2d BD_2 = new Pose2d(53, 32, Math.toRadians(180));
        public static Pose2d BD_3 = new Pose2d(53, 32, Math.toRadians(180));

        public static Pose2d BD_1_OFF = new Pose2d(BD_1.x - 4, BD_1.y + 0.5, BD_1.theta);
        public static Pose2d BD_2_OFF = new Pose2d(BD_2.x - 4, BD_2.y + 0.5, BD_2.theta);
        public static Pose2d BD_3_OFF = new Pose2d(BD_3.x - 4, BD_3.y + 0.5, BD_3.theta);

        public static Pose2d PARK = new Pose2d(48, 16, Math.toRadians(180));
    }
}
