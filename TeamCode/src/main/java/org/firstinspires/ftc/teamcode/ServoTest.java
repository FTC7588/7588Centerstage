package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends CommandOpMode {

    Servo grab1;
    Servo grab2;
    public static double open = 0.53;
    public static double closed = 0.21;

    DcMotorEx intL;
    DcMotorEx intR;

    @Override
    public void initialize() {
        grab1 = hardwareMap.get(Servo.class, "grab1");
        grab2 = hardwareMap.get(Servo.class, "grab2");

        intL = hardwareMap.get(DcMotorEx.class, "intL");
        intR = hardwareMap.get(DcMotorEx.class, "intR");
        intL.setDirection(DcMotorSimple.Direction.REVERSE);

        intL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void run() {
        if (gamepad2.a) {
            grab1.setPosition(closed);
            grab2.setPosition(open);
        } else if (gamepad2.b) {
            grab1.setPosition(open);
            grab2.setPosition(closed);
        } else if (gamepad2.y) {
            grab1.setPosition(1-closed);
            grab2.setPosition(1-closed);
        }
        if (gamepad2.dpad_up) {
            intL.setPower(1);
            intR.setPower(1);
        } else if (gamepad2.dpad_down) {
            intL.setPower(-1);
            intR.setPower(-1);
        } else {
            intL.setPower(0);
            intR.setPower(0);
        }
    }
}

