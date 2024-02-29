package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.poofyutils.CommandOpModeEx;

@TeleOp
@Config
public class kStaticTuner extends CommandOpModeEx {

    public enum Motor {
        FL,
        FR,
        RL,
        RR
    }

    public static Motor motor = Motor.FL;
    public double power;
    public double increment = 0.0001;

    public DcMotorEx fL;
    public DcMotorEx fR;
    public DcMotorEx rL;
    public DcMotorEx rR;

    public DcMotorEx chosenMotor;

    public ElapsedTime testTime;





    @Override
    public void initialize() {
        fL = hardwareMap.get(DcMotorEx.class, "fL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        rL = hardwareMap.get(DcMotorEx.class, "rL");
        rR = hardwareMap.get(DcMotorEx.class, "rR");

        testTime = new ElapsedTime();

        switch (motor) {
            case FL :
                chosenMotor = fL;
                break;
            case FR:
                chosenMotor = fR;
                break;
            case RL:
                chosenMotor = rL;
                break;
            case RR:
                chosenMotor = rR;
                break;
        }

        chosenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chosenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void initLoop() {

    }

    @Override
    public void runOnce() {
        testTime.reset();
    }

    boolean pastA;
    boolean pastB;

    @Override
    public void run() {
//        if (testTime.milliseconds() > 50 && chosenMotor.getCurrentPosition() == 0) {
//            power += increment;
//            chosenMotor.setPower(power);
//            testTime.reset();
//        }

        if (gamepad1.dpad_right) {
            chosenMotor = fR;
        } else if (gamepad1.dpad_left) {
            chosenMotor = fL;
        } else if (gamepad1.dpad_down) {
            chosenMotor = rL;
        } else if (gamepad1.dpad_up) {
            chosenMotor = rR;
        }

        if (gamepad1.a && !pastA) {
            power += increment;
        }
        pastA = gamepad1.a;
        if (gamepad1.b && !pastB) {
            power -= increment;
        }
        pastB = gamepad1.b;

        chosenMotor.setPower(power);

        telemetry.addData("Current Motor", motor);
        telemetry.addData("Testing Power", power);
        telemetry.addData("Motor Position", chosenMotor.getCurrentPosition());
        telemetry.update();
    }
}
