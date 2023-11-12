package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double lCurrent;
    private double rCurrent;
    private double wristCurrent;
    private double pivotCurrent;

    private final InterpLUT shoulderLUT;
    private final InterpLUT wristLUT;
    private final InterpLUT pivotLUT;
    private final InterpLUT touchLUT;

    public ArmSubsystem(RobotHardware robot) {
        this.robot = robot;

        shoulderLUT = new InterpLUT();
        shoulderLUT.add(ARM_SHOULDER_IN_ANGLE, 0);
        shoulderLUT.add(ARM_SHOULDER_OUT_ANGLE, 1);
        shoulderLUT.createLUT();

        wristLUT = new InterpLUT();
        wristLUT.add(ARM_WRIST_IN_ANGLE, 0);
        wristLUT.add(ARM_WRIST_OUT_ANGLE, 1);
        wristLUT.createLUT();

        pivotLUT = new InterpLUT();
        pivotLUT.add(ARM_PIVOT_UP_ANGLE, 0);
        pivotLUT.add(ARM_PIVOT_DOWN_ANGLE, 1);
        pivotLUT.createLUT();

        touchLUT = new InterpLUT();
        touchLUT.add(-1.001, 0);
        touchLUT.add(1.001, 1);
        touchLUT.createLUT();
    }

    public void read() {
//        if (DEBUG_ARM) {
//
//        }

    }

    public void loop() {

    }

    public void write() {

    }

    public void setShoulderAngle(double angle) {
        robot.armL.setPosition(shoulderLUT.get(angle));
        robot.armR.setPosition(shoulderLUT.get(angle));
    }

    public void setShoulderPosition(double position) {
        robot.armL.setPosition(position);
        robot.armR.setPosition(position);
    }

    public void setWristAngle(double angle) {
        robot.armWrist.setPosition(wristLUT.get(angle));
    }

    public void setWristPosition(double position) {
        robot.armWrist.setPosition(position);
    }

    public void setPivotAngle(double angle) {
        robot.armPivot.setPosition(pivotLUT.get(angle));
    }

    public void setPivotPosition(double position) {
        robot.armPivot.setPosition(position);
    }

    public void setShoulderPositionTouch(double position) {
        setShoulderPosition(touchLUT.get(position));
    }

    public double getWristPosition() {
        return robot.armWrist.getPosition();
    }

    public double getShoulderPosition() {
        return robot.armL.getPosition();
    }

//    public double getLeftCurrent() {
//        return
//    }

}
