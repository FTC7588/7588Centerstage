package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.pid.PoofyPIDController;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class ElevatorSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double power;

    private double lPos;
    private double rPos;
    private double avgPos;

    private double lCurrent;
    private double rCurrent;
    private double avgCurrent;

    private double target;

    private final PoofyPIDController controller;

    public ElevatorSubsystem(RobotHardware robot) {
        this.robot = robot;

        controller = new PoofyPIDController(ELE_COEFFS);
    }


    public void read() {
        lPos = robot.eleL.getCurrentPosition();
        rPos = robot.eleR.getCurrentPosition();
        if (DEBUG_ELEVATOR) {
            lCurrent = robot.eleL.getCurrent(CurrentUnit.AMPS);
            rCurrent = robot.eleR.getCurrent(CurrentUnit.AMPS);
        }
    }

    public void loop() {
        avgPos = (lPos + rPos) / 2;
        avgCurrent = (lCurrent + rCurrent) / 2;

        if (ELE_PID) {
            controller.setTargetPosition(target);
            power = controller.calculate(avgPos);
        }
    }

    public void write() {
        robot.eleL.setPower(power);
        robot.eleR.setPower(power);
    }


    public double getLeftPosition() {
        return lPos;
    }

    public double getRightPosition() {
        return rPos;
    }

    public double getPosition() {
        return avgPos;
    }

    public double getTarget() {
        return target;
    }

    public double getPower() {
        return power;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setPower(double power) {
        this.power = power;
    }
}
