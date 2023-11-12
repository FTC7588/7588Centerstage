package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@Autonomous
public class Auto extends OpMode {

    public static   double RUNTIME = 0.3;

    RobotHardware robot;

    private ElapsedTime timer;

    boolean once = false;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        if (!once) {
            timer.reset();
            once = true;
        }

        if (timer.seconds() < RUNTIME) {
            robot.fL.setPower(1);
            robot.fR.setPower(1);
            robot.rL.setPower(1);
            robot.rR.setPower(1);
        } else {
            robot.fL.setPower(0);
            robot.fR.setPower(0);
            robot.rL.setPower(0);
            robot.rR.setPower(0);
        }

    }
}
