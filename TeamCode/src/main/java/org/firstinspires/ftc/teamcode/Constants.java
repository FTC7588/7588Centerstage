package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.poofyutils.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Vector3d;
import org.firstinspires.ftc.teamcode.poofyutils.pid.PoofyPIDCoefficients;

@Config
public class Constants {

    //pid to point
    public static double VELOCITY_THRESHOLD = 0.25;

    public static double INTAKE_JAM_CURRENT = 2.5;
    public static double INTAKE_JAM_REVERSE_TIME = 0.5;

    //control layer buttons
    public static GamepadKeys.Trigger CONTROL_LAYER_2 = GamepadKeys.Trigger.LEFT_TRIGGER;
    public static GamepadKeys.Trigger CONTROL_LAYER_3 = GamepadKeys.Trigger.RIGHT_TRIGGER;

    //debug booleans
    public static boolean DEBUG_GENERAL     = false;
    public static boolean DEBUG_DRIVE       = false;
    public static boolean DEBUG_INTAKE      = false;
    public static boolean DEBUG_ELEVATOR    = false;
    public static boolean DEBUG_ARM         = false;
    public static boolean DEBUG_GRABBER     = false;
    public static boolean DEBUG_VISION      = false;

    //drive constants
    public static PoofyPIDCoefficients X_COEFFS = new PoofyPIDCoefficients(0.08, 0, 0.01, 0, 0, 0.01, 0);
    public static PoofyPIDCoefficients Y_COEFFS = new PoofyPIDCoefficients(0.08, 0, 0.01, 0, 0, 0.01, 0);
    public static PoofyPIDCoefficients THETA_COEFFS = new PoofyPIDCoefficients(1, 0, 0.08, 0, 0, 0.0, 0);

    public static double LOW_SPEED = 0.55;
    public static double HIGH_SPEED = 1;

    public static double DRONE_RELEASE = 0;
    public static double DRONE_HOLD = 1;

    //intake constants
    public static PwmControl.PwmRange INTAKE_RANGE = new PwmControl.PwmRange(750, 2250);
    public static double INTAKE_POWER = 1;

    public static double INT_UP = 0;
    public static double INT_FIVE = 0.18;
    public static double INT_FOUR = 0.235;
    public static double INT_THREE = 0.3;
    public static double INT_TWO = 0.375;
    public static double INT_ONE = 0.435;
    public static double INT_DOWN = 0.42;

    public static double INT_INCREMENT = 0.005;

    //ele constants
    public static PoofyPIDCoefficients ELE_COEFFS = new PoofyPIDCoefficients(0.01, 0, 0);
//    public static TrapezoidProfile.Constraints ELE_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 10);
    public static double ELE_POWER = 1;

    public static boolean ELE_PID = false;

    public static double ELE_MAX = 3600;
    public static double ELE_UP = 1100;
    public static double ELE_MID = 200;
    public static double ELE_DOWN = 0;
    public static double ELE_MIN = 0;

    public static double ELE_HANG = 1025;

    public static double FLOOR_ELE = 200;
    public static double POISED_ELE = 160;
    public static double GRAB_ELE = 35;

    public static double ELE_INCREMENT = 10;

    //arm constants
        //shoulder constants
    public static double ARM_SHOULDER_IN_ANGLE = 60;
    public static double ARM_SHOULDER_OUT_ANGLE = 320;

    public static double ARM_SHOULDER_DEPOSIT = 0.39;
    public static double ARM_SHOULDER_IDLE = 0.71;
    public static double FLOOR_SHOULDER = 0.25;
    public static double POISED_SHOULDER = 0.91;
    public static double GRAB_SHOULDER = 0.892;
    public static double ARM_AUTO = 0.45;
//    public static double GRAB_SHOULDER = 0.89;

        //wrist constants
    public static double ARM_WRIST_IN_ANGLE = -90;
    public static double ARM_WRIST_OUT_ANGLE = 90;

    public static double ARM_WRIST_DEPOSIT = 0.93;
    public static double ARM_WRIST_IDLE = 1;
    public static double ARM_WRIST_TEST = 0.05;
    public static double FLOOR_WRIST = 1;
    public static double POISED_WRIST = 0.19;
    public static double GRAB_WRIST = 0.16;

        //pivot constants
    public static double ARM_PIVOT_UP_ANGLE = 0;
    public static double ARM_PIVOT_DOWN_ANGLE = 180;

    public static double ARM_PIVOT_UP = 0.77;
    public static double ARM_PIVOT_UP_MID = 0.58;
    public static double ARM_PIVOT_MID = 0.485;
    public static double ARM_PIVOT_DOWN_MID = 0.3933;
    public static double ARM_PIVOT_DOWN = 0.21;

    public static double ARM_PIVOT_NORM_LEFT = 0.95;
    public static double ARM_PIVOT_NORM_DOWN = 0.77;
    public static double ARM_PIVOT_NORM_RIGHT = 0.58;

    public static double ARM_PIVOT_NORM_UP = 0.485;

    public static double ARM_PIVOT_ROT_LEFT = 0.3933;
    public static double ARM_PIVOT_ROT_DOWN = 0.21;
    public static double ARM_PIVOT_ROT_RIGHT = 0;

    //grabber constants
    public static double GRABBER_ONE_CLOSED = 0.87;
    public static double GRABBER_ONE_OPEN = 0.65;

    public static double GRABBER_TWO_CLOSED = 0.88;
    public static double GRABBER_TWO_OPEN = 0.66;


    //vision constants
    public static CameraIntrinsics C920_INTRINSICS = new CameraIntrinsics(504.041, 504.041, 307.462, 234.687);
    public static Pose3d C920_POSE = new Pose3d(
            new Vector3d(6.07, 0, 8.05861),
            new Rotation3d(Math.toRadians(-30), 0, Math.toRadians(0))
    );
    public static double C920_EXPOSURE = 11;
    public static int C920_GAIN = 255;

    public static CameraIntrinsics C930_INTRINSICS = new CameraIntrinsics(504.041, 504.041, 307.462, 234.687);
    public static Pose3d C930_POSE = new Pose3d(
            new Vector3d(0, -6.80315, 2.125),
            new Rotation3d(0, 0, Math.toRadians(180))
    );
    public static double C930_EXPOSURE = 11;
    public static int C930_GAIN = 255;

    //backdrop positions
    public static double BLUE_BACKDROP_LEFT = 28;
    public static Pose2d BLUE_BACKDROP = new Pose2d(42, 36, Math.toRadians(360));
    public static double BLUE_BACKDROP_RIGHT = 44;

    public static double RED_BACKDROP_LEFT = -28;
    public static Pose2d RED_BACKDROP = new Pose2d(-42, 36, Math.toRadians(360));
    public static double RED_BACKDROP_RIGHT = -44;

    public static org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d FOLLOW_POSE = new Pose2d(42, 36, Math.toRadians(360));

}
