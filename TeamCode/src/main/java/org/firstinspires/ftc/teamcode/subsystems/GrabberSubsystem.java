package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class GrabberSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double leftPos = Constants.GRABBER_ONE_OPEN;
    private double rightPos = Constants.GRABBER_TWO_OPEN;

    private enum GrabberState {
        OPEN,
        CLOSED
    }

    private GrabberState leftGrabberState;
    private GrabberState rightGrabberState;

    public GrabberSubsystem(RobotHardware robot) {
        this.robot = robot;
        leftGrabberState = GrabberState.CLOSED;
        rightGrabberState = GrabberState.CLOSED;
    }

    public void read() {

    }

    public void loop() {
        if (leftGrabberState == GrabberState.OPEN) {
            leftPos = Constants.GRABBER_ONE_CLOSED;
        } else if (leftGrabberState == GrabberState.CLOSED) {
            leftPos = Constants.GRABBER_ONE_OPEN;
        }

        if (rightGrabberState == GrabberState.OPEN) {
            rightPos = Constants.GRABBER_TWO_CLOSED;
        } else if (rightGrabberState == GrabberState.CLOSED) {
            rightPos = Constants.GRABBER_TWO_OPEN;
        }
    }

    public void write() {
        robot.grab1.setPosition(leftPos);
        robot.grab2.setPosition(rightPos);
    }

    public void setGrabberPosition(double pos) {
        leftPos = pos;
        rightPos = pos;
    }

    public void setGrabberPositions(double pos1, double pos2) {
        leftPos = pos1;
        rightPos = pos2;
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

    public void toggleLeftGrabber() {
        if (leftGrabberState == GrabberState.CLOSED) {
            leftGrabberState = GrabberState.OPEN;
        } else if (leftGrabberState == GrabberState.OPEN) {
            leftGrabberState = GrabberState.CLOSED;
        }
    }

    public void toggleRightGrabber() {
        if (rightGrabberState == GrabberState.CLOSED) {
            rightGrabberState = GrabberState.OPEN;
        } else if (rightGrabberState == GrabberState.OPEN) {
            rightGrabberState = GrabberState.CLOSED;
        }
    }

    public void toggleGrabbers() {
        if (leftGrabberState == GrabberState.CLOSED) {
            leftGrabberState = GrabberState.OPEN;
            rightGrabberState = GrabberState.OPEN;
        } else if (leftGrabberState == GrabberState.OPEN) {
            leftGrabberState = GrabberState.CLOSED;
            rightGrabberState = GrabberState.CLOSED;
        }
    }

    public boolean isClosed() {
        return (leftPos == Constants.GRABBER_ONE_CLOSED && rightPos == Constants.GRABBER_ONE_CLOSED);
    }
}
