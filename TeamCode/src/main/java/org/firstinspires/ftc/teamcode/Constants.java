package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.utils.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector3d;
import org.firstinspires.ftc.teamcode.utils.pid.PoofyPIDCoefficients;

@Config
public class Constants {

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
    public static PoofyPIDCoefficients X_COEFFS = new PoofyPIDCoefficients(0, 0, 0);
    public static PoofyPIDCoefficients Y_COEFFS = new PoofyPIDCoefficients(0, 0, 0);
    public static PoofyPIDCoefficients THETA_COEFFS = new PoofyPIDCoefficients(0.005, 0, 0);

    public static double LOW_SPEED = 0.4;
    public static double HIGH_SPEED = 1;

    //intake constants
    public static PwmControl.PwmRange INTAKE_RANGE = new PwmControl.PwmRange(750, 2250);
    public static double INTAKE_POWER = 0.6;

    public static double INT_DOWN = 0.375;
    public static double INT_UP = 0.25;

    //ele constants
    public static PoofyPIDCoefficients ELE_COEFFS = new PoofyPIDCoefficients(0.01, 0, 0);
//    public static TrapezoidProfile.Constraints ELE_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 10);
    public static double ELE_POWER = 1;

    public static boolean ELE_PID = true;

    public static double ELE_UP = 1000;
    public static double ELE_MID = 200;
    public static double ELE_DOWN = 0;

    //arm constants
        //shoulder constants
    public static double ARM_SHOULDER_IN_ANGLE = 60;
    public static double ARM_SHOULDER_OUT_ANGLE = 320;

    public static double ARM_SHOULDER_DEPOSIT = 0.4;
    public static double ARM_SHOULDER_IDLE = 0.7;
    public static double ARM_SHOULDER_POISED = 0.91;
    public static double ARM_SHOULDER_GRAB = 0.95;

        //wrist constants
    public static double ARM_WRIST_IN_ANGLE = -90;
    public static double ARM_WRIST_OUT_ANGLE = 90;

    public static double ARM_WRIST_DEPOSIT = 0.3;
    public static double ARM_WRIST_IDLE = 0.7;
    public static double ARM_WRIST_POISED = 0.8;
    public static double ARM_WRIST_GRAB = 0.85;

        //pivot constants
    public static double ARM_PIVOT_UP_ANGLE = 0;
    public static double ARM_PIVOT_DOWN_ANGLE = 180;

    public static double ARM_PIVOT_UP = 1;
    public static double ARM_PIVOT_DOWN = 0;

    //grabber constants
    public static double GRAB_CLOSED = 0.825;
    public static double GRAB_OPEN = 0.53;


    //vision constants
    public static CameraIntrinsics C920_INTRINSICS = new CameraIntrinsics(504.041, 504.041, 307.462, 234.687);
    public static Pose3d C920_POSE = new Pose3d(
            new Vector3d(3.125, -7.125, 3.5),
            new Rotation3d(0, 0, Math.toRadians(0))
    );

}
