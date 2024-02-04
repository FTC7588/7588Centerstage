package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    public static boolean PIVOT_ROTATE = true;

    private double shoulderPos = POISED_SHOULDER;
    private double wristPos;
    private double pivotPos;

    private final InterpLUT touchLUT;

    public PivotRotatedState pivotRotatedState = PivotRotatedState.NORMAL;
    public PivotPositionState pivotPositionState = PivotPositionState.UP;
    public ShoulderState shoulderState = ShoulderState.IDLE;

    public PivotRotatedState lastPivotRotated;
    public PivotPositionState lastPivotPosition;
    public ShoulderState lastArmState;

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
        if (true) {
            switch (pivotRotatedState) {
                case NORMAL:
                    switch (pivotPositionState) {
                        case UP:
                            pivotPos = Constants.ARM_PIVOT_NORM_UP;
                            break;
                        case MID:
                            pivotPos = ARM_PIVOT_NORM_DOWN;
                            break;
                        case LEFT:
                            pivotPos = ARM_PIVOT_NORM_LEFT;
                            break;
                        case RIGHT:
                            pivotPos = ARM_PIVOT_NORM_RIGHT;
                            break;
                    }
                    break;
                case ROTATED:
                    switch (pivotPositionState) {
                        case UP:
                            pivotPos = ARM_PIVOT_NORM_UP;
                            break;
                        case MID:
                            pivotPos = ARM_PIVOT_ROT_DOWN;
                            break;
                        case LEFT:
                            pivotPos = ARM_PIVOT_ROT_LEFT;
                            break;
                        case RIGHT:
                            pivotPos = ARM_PIVOT_ROT_RIGHT;
                            break;
                    }
            }
        }

        lastPivotPosition = pivotPositionState;
        lastPivotRotated = pivotRotatedState;

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

    public void setPivotStates(PivotRotatedState rot, PivotPositionState pos) {
        pivotPositionState = pos;
        pivotRotatedState = rot;
    }

    public void toggleRotated() {
        if (pivotRotatedState == PivotRotatedState.NORMAL) {
            pivotRotatedState = PivotRotatedState.ROTATED;
        } else if (pivotRotatedState == PivotRotatedState.ROTATED) {
            pivotRotatedState = PivotRotatedState.NORMAL;
        }
    }


    public enum PivotRotatedState {
        NORMAL,
        ROTATED
    }

    public enum PivotPositionState {
        LEFT,
        MID,
        RIGHT,
        UP
    }

    public enum ShoulderState {
        IDLE,
        GRAB,
        DEPOSIT,
        AUTO
    }

    public enum WristState {

    }

}
