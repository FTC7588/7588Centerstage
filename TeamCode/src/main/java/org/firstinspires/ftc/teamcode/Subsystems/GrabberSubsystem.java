package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class GrabberSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    public GrabberSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    public void read() {

    }

    public void loop() {

    }

    public void write() {

    }

    public void setGrabberPosition(double pos) {
        robot.grab1.setPosition(pos);
        robot.grab2.setPosition(pos);
    }

}
