package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class GrabberSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double pos;

    public GrabberSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    public void read() {

    }

    public void loop() {

    }

    public void write() {
        robot.grab1.setPosition(pos);
        robot.grab2.setPosition(pos);
    }

    public void setGrabberPosition(double pos) {
        this.pos = pos;
    }

    public double getPos() {
        return pos;
    }
}
