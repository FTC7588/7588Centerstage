package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private final double mod = 384.5;
    private double modPos;

    private final ElapsedTime loadedTime;

    private boolean runOnce = false;
    private boolean loadOnce = false;
    private boolean isEnabled = true;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
        controller = new PoofyPIDController(new PoofyPIDCoefficients(0.01, 0, 0));
        loadedTime = new ElapsedTime();
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

//        disableIfLoaded();
    }

    public void write() {
        if (isEnabled) {
            if (power == 0) {
                robot.intLMotor.setPower(pidPower);
                robot.intRMotor.setPower(pidPower);

            } else {
                robot.intLMotor.setPower(power);
                robot.intRMotor.setPower(power);

            }
        }
        robot.intLServo.setPosition(servoPos);
        robot.intRServo.setPosition(servoPos);
    }


    public boolean isBackPixelLoaded() {
        return ((DistanceSensor) robot.backCS).getDistance(DistanceUnit.CM) < 1;
    }

    public boolean isFrontPixelLoaded() {
        return ((DistanceSensor) robot.frontCS).getDistance(DistanceUnit.CM) < 1;
    }

    public boolean isLoaded() {
        return (isBackPixelLoaded() && isFrontPixelLoaded());
    }

    public void hasBeenLoaded(double millis) {
        if (isLoaded()) {
            if (!loadOnce) {
                loadOnce = true;
                loadedTime.reset();
            }
            if (loadedTime.seconds() > millis) {
                isEnabled = false;
            }
        } else {
            isEnabled = true;
        }
    }

    public double getPower() {
        return power;
    }

    public double getServoPosition() {
        return servoPos;
    }

    public double getIntakePosition() {
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
