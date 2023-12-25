package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.poofyutils.pid.PoofyPIDCoefficients;
import org.firstinspires.ftc.teamcode.poofyutils.pid.PoofyPIDController;

public class IntakeSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double lCurrent;
    private double rCurrent;

    private double power;
    private double pidPower;
    private double servoPos;
    private double elePos;
    private double eleTarget;

    private PoofyPIDController controller;

    private final double mod = 128.16;
    private double modPos;

    private boolean runOnce = false;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
        controller = new PoofyPIDController(new PoofyPIDCoefficients(0.01, 0, 0));
    }

    public void read() {
        if (Constants.DEBUG_INTAKE) {
            lCurrent = robot.intLMotor.getCurrent(AMPS);
            rCurrent = robot.intRMotor.getCurrent(AMPS);
        }
    }

    public void loop() {
        elePos = robot.intLMotor.getCurrentPosition();
        modPos = elePos % mod;
        if (power == 0 && runOnce) {
            eleTarget = elePos + (mod-modPos);
            controller.setTargetPosition(eleTarget);
            runOnce = false;
        } else if (runOnce) {
            runOnce = false;
        }
        pidPower = controller.calculate(elePos);
    }

    public void write() {
        if (power == 0) {
            robot.intLMotor.setPower(pidPower);
            robot.intRMotor.setPower(pidPower);

        } else {
            robot.intLMotor.setPower(power);
            robot.intRMotor.setPower(power);

        }
        robot.intLServo.setPosition(servoPos);
        robot.intRServo.setPosition(servoPos);
    }


    public boolean isBackPixelLoaded() {
        return ((DistanceSensor) robot.backCS).getDistance(DistanceUnit.CM) < 10;
    }

    public boolean isFrontPixelLoaded() {
        return ((DistanceSensor) robot.frontCS).getDistance(DistanceUnit.CM) < 10;
    }

    public double getPower() {
        return power;
    }

    public double getServoPosition() {
        return servoPos;
    }

    public double getElePosition() {
        return elePos;
    }

    public double getModPosition() {
        return modPos;
    }

    public double getTarget() {
        return eleTarget;
    }

    public double getlCurrent() {
        return lCurrent;
    }

    public double getrCurrent() {
        return rCurrent;
    }

    public void setPower(double power) {
        this.power = power;
        runOnce = true;
    }

    public void setPosition(double pos) {
        this.servoPos = pos;
    }
}
