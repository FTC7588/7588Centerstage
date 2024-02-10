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
    private double intPos;
    private double intTarget;

    private PoofyPIDController controller;

    private final double mod = 384.5;
    private double modPos;

    private final ElapsedTime loadedTime;

    private boolean runOnce = false;
    private boolean loadOnce = false;
    private boolean isEnabled = true;


    private boolean isJammed = false;
    private ElapsedTime jamTimer = new ElapsedTime();

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
        controller = new PoofyPIDController(new PoofyPIDCoefficients(0.01, 0, 0));
        loadedTime = new ElapsedTime();
    }

    public void read() {
        if (Constants.DEBUG_INTAKE || RobotHardware.SMART_INTAKE) {
            lCurrent = robot.intLMotor.getCurrent(AMPS);
            rCurrent = robot.intRMotor.getCurrent(AMPS);
        }
    }

    public void loop() {
        if (!RobotHardware.SMART_INTAKE) {
            intPos = robot.intLMotor.getCurrentPosition();
            modPos = intPos % mod;
            if (power == 0 && runOnce) {
                intTarget = intPos + (mod-modPos);
                controller.setTargetPosition(intTarget);
                runOnce = false;
            } else if (runOnce) {
                runOnce = false;
            }
            pidPower = controller.calculate(intPos);
        } else {
            //jam logic
            if (isJammed) {
                if (jamTimer.seconds() < Constants.INTAKE_JAM_REVERSE_TIME) {
                    power = Constants.INTAKE_POWER;
                } else {
                    isJammed = false;
                    power = -Constants.INTAKE_POWER;
                }
            } else {
                intPos = robot.intLMotor.getCurrentPosition();
                modPos = intPos % mod;
                if (power == 0 && runOnce) {
                    intTarget = intPos + (mod-modPos);
                    controller.setTargetPosition(intTarget);
                    runOnce = false;
                } else if (runOnce) {
                    runOnce = false;
                }
                pidPower = controller.calculate(intPos);
            }

            if ((lCurrent + rCurrent) / 2 >= Constants.INTAKE_JAM_CURRENT && !isJammed) {
                isJammed = true;
                jamTimer.reset();
            }
        }
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

    public double getAverageCurrent() {
        return (lCurrent + rCurrent) / 2;
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
        return intPos;
    }

    public double getModPosition() {
        return modPos;
    }

    public double getTarget() {
        return intTarget;
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
