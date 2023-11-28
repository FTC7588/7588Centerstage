package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ButtonTesting extends CommandOpMode {


    @Override
    public void initialize() {

    }

    @Override
    public void run() {
        telemetry.addData("Touchpad", gamepad1.touchpad);
        telemetry.addData("Touchpad Finger 1", gamepad1.touchpad_finger_1);
        telemetry.addData("Touchpad Finger 2", gamepad1.touchpad_finger_2);
        telemetry.update();
    }
}
