package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.poofyutils.pid.PoofyPIDController;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class ElevatorSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double power;

    private double pos;

    private double lCurrent;
    private double rCurrent;

    private double target;

    private final PoofyPIDController controller;

    public ElevatorSubsystem(RobotHardware robot) {
        this.robot = robot;

        controller = new PoofyPIDController(ELE_COEFFS);
        robot.eleL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.eleR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.eleL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.eleR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void read() {
        pos = robot.eleL.getCurrentPosition();
        if (DEBUG_ELEVATOR) {
            lCurrent = robot.eleL.getCurrent(CurrentUnit.AMPS);
            rCurrent = robot.eleR.getCurrent(CurrentUnit.AMPS);
        }
    }

    public void loop() {
        if (ELE_PID) {
            controller.setTargetPosition(target);
            power = controller.calculate(pos);
        }
    }

    public void write() {
        robot.eleL.setPower(power);
        robot.eleR.setPower(power);
    }


    public double getPosition() {
        return pos;
    }

    public double getLeftCurrent() {
        return lCurrent;
    }

    public double getRightCurrent() {
        return rCurrent;
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

    public void addTarget(double targetPlus) {
        target += targetPlus;
        if (target > ELE_UP) {
            target = ELE_UP;
        } else if (target <= 0) {
            target = 0;
        }
    }

    public void setPower(double power) {
        this.power = power;
    }
}
