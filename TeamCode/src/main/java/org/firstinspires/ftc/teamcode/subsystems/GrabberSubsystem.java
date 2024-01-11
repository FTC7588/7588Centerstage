package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class GrabberSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double leftPos;
    private double rightPos;

    public GrabberSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    public void read() {

    }

    public void loop() {

    }

    public void write() {
        robot.grab1.setPosition(leftPos);
        robot.grab2.setPosition(rightPos);
    }

    public void setGrabberPosition(double pos) {
        leftPos = pos;
        rightPos = pos;
    }

    public void setLeftGrabberPosition(double pos) {
        leftPos = pos;
    }

    public void setRightGrabberPosition(double pos) {
        rightPos = pos;
    }

    public double getLeftPos() {
        return leftPos;
    }

    public double getRightPos() {
        return rightPos;
    }

    public boolean isClosed() {
        return (leftPos == Constants.GRABBER_CLOSED && rightPos == Constants.GRABBER_CLOSED);
    }
}
