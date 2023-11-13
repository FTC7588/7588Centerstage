package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class IntakeSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double lCurrent;
    private double rCurrent;

    private double power;
    private double pos;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    public void read() {
        if (Constants.DEBUG_INTAKE) {
            lCurrent = robot.intLMotor.getCurrent(AMPS);
            rCurrent = robot.intRMotor.getCurrent(AMPS);
        }
    }

    public void loop() {

    }

    public void write() {
        robot.intLMotor.setPower(power);
        robot.intRMotor.setPower(power);

        robot.intLServo.setPosition(pos);
        robot.intRServo.setPosition(pos);
    }


    public double getPower() {
        return power;
    }

    public double getPosition() {
        return pos;
    }

    public double getlCurrent() {
        return lCurrent;
    }

    public double getrCurrent() {
        return rCurrent;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setPosition(double pos) {
        this.pos = pos;
    }
}
