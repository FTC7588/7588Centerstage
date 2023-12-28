package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import static org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys.Button.*;

@TeleOp
public class ArmIntakeTesting extends BaseOpMode {

    @Override
    public void initialize() {
        super.initialize();

        gp1(DPAD_DOWN, 1).whenActive(shoulderIncDown);
        gp1(DPAD_UP, 1).whenActive(shoulderIncUp);

        gp1(A, 1).whenActive(wristIncDown);
        gp1(Y, 1).whenActive(wristIncUp);

//        gp1(B, 1).whenActive(intakeIn).whenInactive(intakeIdle);
//        gp1(X, 1).whenActive(intakeOut).whenInactive(intakeIdle);

//        intakeIdle.schedule();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Shoulder Position", armSS.getShoulderPosition());
        telemetry.addData("Wrist Position", armSS.getWristPosition());
        telemetry.update();
    }
}
