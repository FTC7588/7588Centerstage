package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.GrabberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.filters.MovingAverage;
import org.firstinspires.ftc.teamcode.utils.gamepads.GamepadTrigger;
import org.firstinspires.ftc.teamcode.utils.gamepads.TriggerGamepadEx;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class BaseOpMode extends CommandOpMode {

    protected final RobotHardware robot = RobotHardware.getInstance();

    protected DrivetrainSubsystem driveSS;
    protected IntakeSubsystem intakeSS;
    protected ElevatorSubsystem eleSS;
    protected ArmSubsystem armSS;
    protected GrabberSubsystem grabSS;

    protected GamepadEx driver;
    protected GamepadEx operator;
    protected TriggerGamepadEx driverEx;
    protected TriggerGamepadEx operatorEx;

    protected MultipleTelemetry tele;

    protected double loopTime;
    protected MovingAverage loopAvg;

    @Override
    public void initialize() {
        //init motors and servos
        robot.init(hardwareMap);

        //init subsystems
        driveSS = new DrivetrainSubsystem(robot);
        intakeSS = new IntakeSubsystem(robot);
        eleSS = new ElevatorSubsystem(robot);
        armSS = new ArmSubsystem(robot);
        grabSS = new GrabberSubsystem(robot);

        //gamepads
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        driverEx = new TriggerGamepadEx(gamepad1, driver);
        operatorEx = new TriggerGamepadEx(gamepad2, operator);

        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        loopAvg = new MovingAverage(50);
    }

    @Override
    public void run() {
        super.run();

        if (DEBUG_GENERAL) {
            tal("=== GENERAL DEBUG INFO ===");
            tad("Loop Average", loopAvg.getAverage());
            loopAvg.addNumber(System.currentTimeMillis() - loopTime);
            loopTime = System.currentTimeMillis();
            tal();
        }

        if (DEBUG_DRIVE) {
            tal("=== DRIVE DEBUG INFO ===");
            tad("Drive Mode", driveSS.getMode());
            tad("Heading", driveSS.getHeading());
            tad("Max Speed", driveSS.getMaxSpeed());
            tad("Heading Lock Enabled", driveSS.getHeadingLock());
            tad("Heading Lock Target", driveSS.getHeadingLockTarget());
            tal();
        }

        if (DEBUG_INTAKE) {
            tal("=== INTAKE DEBUG INFO ===");
            tad("Intake Power", intakeSS.getPower());
            tad("Intake Position", intakeSS.getPosition());
        }

        if (DEBUG_ELEVATOR) {
            tal("=== ELEVATOR DEBUG INFO ===");
            tad("Elevator Power", eleSS.getPower());
            tad("Elevator Left Position", eleSS.getLeftPosition());
            tad("Elevator Right Position", eleSS.getRightPosition());
            tad("Elevator Average Position", eleSS.getPosition());
            tad("Elevator Target", eleSS.getTarget());
        }

        if (DEBUG_ARM) {

        }

        if (DEBUG_GRABBER) {

        }

        if (DEBUG_VISION) {
            tal("=== VISION DEBUG INFO ===");
            tad("Tag Pose", driveSS.getTagLocalizer().getTagPose());
            tad("Tag Readings", driveSS.getTagLocalizer().getTagReadings());
            tad("Camera Pose", driveSS.getTagLocalizer().getCameraPose());
            tad("Robot Pose", driveSS.getRobotPose());
        }
    }


    protected void tau() {
        tele.update();
    }
    protected void tal() {
        tele.addLine();
    }
    protected void tal(String caption) {
        tele.addLine(caption);
    }
    protected void tad(String caption, Object value) {
        tele.addData(caption, value);
    }

    protected GamepadButton gp1(GamepadKeys.Button button) {
        return driver.getGamepadButton(button);
    }

    protected GamepadTrigger gp1(GamepadKeys.Trigger trigger) {
        return driverEx.getGamepadTrigger(trigger);
    }

    protected Trigger gp1(GamepadKeys.Button button, int layer) {
        if (layer == 1) {
            return driver.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return driver.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return driver.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return driver.getGamepadButton(button);
        }
    }

    protected Trigger gp1(GamepadKeys.Trigger trigger, int layer) {
        if (layer == 1) {
            return driverEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return driverEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return driverEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return driverEx.getGamepadTrigger(trigger);
        }
    }



    protected GamepadButton gp2(GamepadKeys.Button button) {
        return operator.getGamepadButton(button);
    }

    protected GamepadTrigger gp2(GamepadKeys.Trigger trigger) {
        return operatorEx.getGamepadTrigger(trigger);
    }

    protected Trigger gp2(GamepadKeys.Button button, int layer) {
        if (layer == 1) {
            return operator.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return operator.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return operator.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return operator.getGamepadButton(button);
        }
    }

    protected Trigger gp2(GamepadKeys.Trigger trigger, int layer) {
        if (layer == 1) {
            return operatorEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return operatorEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return operatorEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return operatorEx.getGamepadTrigger(trigger);
        }
    }
}
