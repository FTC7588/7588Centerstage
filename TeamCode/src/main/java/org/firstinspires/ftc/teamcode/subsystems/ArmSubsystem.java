package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double shoulderPos = POISED_SHOULDER;
    private double wristPos;
    private double pivotPos;

    private final InterpLUT touchLUT;

    public PivotRotatedState pivotRotatedState = PivotRotatedState.NORMAL;
    public PivotPositionState pivotPositionState = PivotPositionState.UP;
    public ShoulderState shoulderState = ShoulderState.IDLE;
    public WristState wristState = WristState.IDLE;
    public ArmState armState = ArmState.IDLE;

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
        //pivot logic
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

        //pivot logic
        switch (wristState) {
            case IDLE:
                wristPos = ARM_WRIST_IDLE;
                break;
            case GRAB:
                wristPos = GRAB_WRIST;
                break;
            case DEPOSIT:
                wristPos = ARM_WRIST_DEPOSIT;
                break;
            case AUTO:
                wristPos = FLOOR_WRIST;
                break;
        }

        //shoulder logic
        switch (shoulderState) {
            case IDLE:
                shoulderPos = POISED_SHOULDER;
                break;
            case GRAB:
                shoulderPos = GRAB_SHOULDER;
                break;
            case DEPOSIT:
                shoulderPos = ARM_SHOULDER_DEPOSIT;
                break;
            case AUTO:
                shoulderPos = FLOOR_SHOULDER;
                break;
        }

        if (shoulderPos == POISED_SHOULDER || shoulderPos == GRAB_SHOULDER) {
            pivotPositionState = PivotPositionState.UP;
            pivotRotatedState = PivotRotatedState.NORMAL;
        }



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

    public void setPivotRotationState(PivotRotatedState rot) {
        if (shoulderPos < 0.75) {
            pivotRotatedState = rot;
        }
    }

    public void setPivotPositionState(PivotPositionState pos) {
        if (shoulderPos < 0.75) {
            pivotPositionState = pos;
        }
    }

    public void toggleRotated() {
        if (shoulderPos < 0.75) {
            if (pivotRotatedState == PivotRotatedState.NORMAL) {
                pivotRotatedState = PivotRotatedState.ROTATED;
            } else if (pivotRotatedState == PivotRotatedState.ROTATED) {
                pivotRotatedState = PivotRotatedState.NORMAL;
            }
        }
    }

    public void setArmState(ArmState state) {
        armState = state;
        switch (armState) {
            case IDLE:
                wristState = WristState.IDLE;
                shoulderState = ShoulderState.IDLE;
                break;
            case GRAB:
                wristState = WristState.GRAB;
                shoulderState = ShoulderState.GRAB;
                break;
            case DEPOSIT:
                wristState = WristState.DEPOSIT;
                shoulderState = ShoulderState.DEPOSIT;
                break;
            case AUTO:
                wristState = WristState.AUTO;
                shoulderState = ShoulderState.AUTO;
        }
    }

    public void setWristState(WristState state) {
        wristState = state;
    }

    public void setShoulderState(ShoulderState state) {
        shoulderState = state;
    }

    public void toggleArmState() {

    }

    public ShoulderState getShoulderState() {
        return shoulderState;
    }

    public WristState getWristState() {
        return wristState;
    }

    public ArmState getArmState() {
        return armState;
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
        IDLE,
        GRAB,
        DEPOSIT,
        AUTO
    }

    public enum ArmState {
        IDLE,
        GRAB,
        DEPOSIT,
        AUTO
    }

}
