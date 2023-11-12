package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class IntakeSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double power;
    private double pos;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    public void read() {

    }

    public void loop() {

    }

    public void write() {
        robot.intLMotor.setPower(power);
        robot.intRMotor.setPower(power);

        robot.intLServo.setPosition(pos);
        robot.intRServo.setPosition(pos);
    }


    public double getPower() {
        return power;
    }

    public double getPosition() {
        return pos;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setPosition(double pos) {
        this.pos = pos;
    }
}
