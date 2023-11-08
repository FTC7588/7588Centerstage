package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Config
public class ServoTest extends CommandOpMode {

    Servo grab1;
    Servo grab2;
    public static double open = 0.53;
    public static double closed = 0.21;

//    AnalogInput sensor;

    DcMotorEx intL;
    DcMotorEx intR;

    @Override
    public void initialize() {
        grab1 = hardwareMap.get(Servo.class, "grab1");
        grab2 = hardwareMap.get(Servo.class, "grab2");

        grab2.setDirection(Servo.Direction.REVERSE);

//        grab1.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        grab2.setPwmRange(new PwmControl.PwmRange(500, 2500));

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
            grab2.setPosition(closed);
        } else if (gamepad2.b) {
            grab1.setPosition(open);
            grab2.setPosition(open);
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

        telemetry.addData("a", gamepad2.a);

//        telemetry.addData("angle", sensor.getVoltage());
//        telemetry.addData("data", sensor.getConnectionInfo());
//        telemetry.addData("max", sensor.getMaxVoltage());
        telemetry.update();
    }
}
