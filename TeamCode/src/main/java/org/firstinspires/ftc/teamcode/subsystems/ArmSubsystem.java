package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double shoulderPos = POISED_SHOULDER;
    private double wristPos;
    private double pivotPos;

    private final InterpLUT touchLUT;

    public ArmSubsystem(RobotHardware robot) {
        this.robot = robot;

        touchLUT = new InterpLUT();
        touchLUT.add(-1.001, 0);
        touchLUT.add(1.001, 1);
        touchLUT.createLUT();
    }

    public void read() {

    }

    public void loop() {

    }

    public void write() {
        robot.armL.setPosition(shoulderPos);
        robot.armR.setPosition(shoulderPos);
        robot.armWrist.setPosition(wristPos);
        robot.armPivot.setPosition(pivotPos);
    }

    public void setShoulderPosition(double position) {
        shoulderPos = position;
    }

    public void setWristPosition(double position) {
        wristPos = position;
    }

    public void setPivotPosition(double position) {
        pivotPos = position;
    }

    public void setShoulderPositionTouch(double position) {
        setShoulderPosition(touchLUT.get(position));
    }

    public double getShoulderPosition() {
        return shoulderPos;
    }

    public double getWristPosition() {
        return wristPos;
    }

    public double getPivotPosition() {
        return pivotPos;
    }

}
