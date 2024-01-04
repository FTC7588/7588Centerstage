package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.arm.IncrementShoulderPosition;
import org.firstinspires.ftc.teamcode.commands.arm.IncrementWristPosition;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetEleArmPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetPivotPosition;
import org.firstinspires.ftc.teamcode.commands.arm.SetShoulderPosition;
import org.firstinspires.ftc.teamcode.commands.arm.SetWristPosition;
import org.firstinspires.ftc.teamcode.commands.drive.BackdropTagSlide;
import org.firstinspires.ftc.teamcode.commands.drive.EnableHeadingLock;
import org.firstinspires.ftc.teamcode.commands.drive.FieldCentric;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTag;
import org.firstinspires.ftc.teamcode.commands.drive.RobotCentric;
import org.firstinspires.ftc.teamcode.commands.drive.SetHeadingLock;
import org.firstinspires.ftc.teamcode.commands.drive.SetMaxSpeed;
import org.firstinspires.ftc.teamcode.commands.elevator.IncrementElevatorTarget;
import org.firstinspires.ftc.teamcode.commands.elevator.SetElevatorPower;
import org.firstinspires.ftc.teamcode.commands.elevator.SetElevatorTarget;
import org.firstinspires.ftc.teamcode.commands.grabber.SetGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.grabber.SetLeftGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.grabber.SetRightGrabberPosition;
import org.firstinspires.ftc.teamcode.commands.intake.IncrementIntakeAngle;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeAngle;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commands.intake.VariableIntakeAngle;
import org.firstinspires.ftc.teamcode.poofyutils.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.poofyutils.enums.Alliance;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.PoofyGamepadEx;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.readers.GamepadButton;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.AutoDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrabberSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.poofyutils.filters.MovingAverage;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadTrigger;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class BaseOpMode extends CommandOpModeEx {

    //singletons
    protected final RobotHardware robot = RobotHardware.getInstance();
    protected FtcDashboard dashboard = FtcDashboard.getInstance();

    //subsystems
    protected AutoDrivetrainSubsystem autoDriveSS;
    protected SampleMecanumDrive rrDrive;
    protected DrivetrainSubsystem driveSS;
    protected IntakeSubsystem intakeSS;
    protected ElevatorSubsystem eleSS;
    protected ArmSubsystem armSS;
    protected GrabberSubsystem grabSS;

    //commands
    protected RobotCentric robotCentric;
    protected FieldCentric fieldCentric;
    protected FollowTag followTag;
    protected BackdropTagSlide followBackdrop;

    protected SetMaxSpeed lowSpeed;
    protected SetMaxSpeed highSpeed;

    protected EnableHeadingLock enabledHeadingLock;
    protected EnableHeadingLock disableHeadingLock;

    protected SetHeadingLock forwardLock;
    protected SetHeadingLock leftLock;
    protected SetHeadingLock backLock;
    protected SetHeadingLock rightLock;

    protected SetIntakePower intakeIn;
    protected SetIntakePower intakeIdle;
    protected SetIntakePower intakeOut;

    protected SetIntakeAngle intakeUp;
    protected SetIntakeAngle intakeDown;

    protected IncrementIntakeAngle incrementIntakeUp;
    protected IncrementIntakeAngle incrementIntakeDown;

    protected VariableIntakeAngle variableIntakeAngle;

    protected SetElevatorPower eleUp;
    protected SetElevatorPower eleIdle;
    protected SetElevatorPower eleDown;

    protected SetElevatorTarget eleTargetUp;
    protected SetElevatorTarget eleTargetMid;
    protected SetElevatorTarget eleTargetDown;
    protected SetElevatorTarget eleTargetHang;

    protected IncrementElevatorTarget eleIncUp;
    protected IncrementElevatorTarget eleIncDown;

    protected SetShoulderPosition shoulderPosDeposit;
    protected SetShoulderPosition shoulderPosPoised;
    protected SetShoulderPosition shoulderPosGrab;
    protected SetShoulderPosition shoulderPosIdle;

    protected IncrementShoulderPosition shoulderIncUp;
    protected IncrementShoulderPosition shoulderIncDown;

    protected SetWristPosition wristPosDown;
    protected SetWristPosition wristPosPoised;
    protected SetWristPosition wristPosGrab;
    protected SetWristPosition wristPosUp;

    protected IncrementWristPosition wristIncDown;
    protected IncrementWristPosition wristIncUp;

    protected SetPivotPosition pivotPosDown;
    protected SetPivotPosition pivotPosMid;
    protected SetPivotPosition pivotPosUp;

    protected SetGrabberPosition grabbersClosed;
    protected SetGrabberPosition grabbersOpen;

    protected SetLeftGrabberPosition grabberLeftClose;
    protected SetLeftGrabberPosition grabberLeftOpen;
    protected SetRightGrabberPosition grabberRightClose;
    protected SetRightGrabberPosition grabberRightOpen;

    protected SetArmPositions armInit;
    protected SetEleArmPositions armDown;
    protected SetEleArmPositions armDeposit;
    protected SetEleArmPositions armIdle;
    protected SetEleArmPositions armPoised;
    protected SetEleArmPositions armGrab;
    protected SetEleArmPositions armBack;

    protected SequentialCommandGroup armDepositGroup;
    protected SequentialCommandGroup armIdleGroup;

    protected ConditionalCommand autoGrab;

    protected PoofyGamepadEx driver;
    protected PoofyGamepadEx operator;

    protected MultipleTelemetry tele;

    protected double loopTime;
    protected MovingAverage loopAvg;

    protected boolean auto = false;
    protected Alliance alliance;

    @Override
    public void initialize() {
        //init motors and servos
        robot.init(hardwareMap);

        //init subsystems
        if (auto) {
            rrDrive = new SampleMecanumDrive(hardwareMap);
            autoDriveSS = new AutoDrivetrainSubsystem(robot, rrDrive, false);
        } else {
            driveSS = new DrivetrainSubsystem(robot);
        }

        intakeSS = new IntakeSubsystem(robot);
        eleSS = new ElevatorSubsystem(robot);
        armSS = new ArmSubsystem(robot);
        grabSS = new GrabberSubsystem(robot);

        //drive commands
        robotCentric = new RobotCentric(
                driveSS,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()
        );

        fieldCentric = new FieldCentric(
                driveSS,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()
        );

        followTag = new FollowTag(
                driveSS,
                FOLLOW_POSE
        );

//        followBackdrop = new BackdropTagSlide(
//                driveSS,
//                () -> driver.getTouchX(),
//                alliance
//        );

        lowSpeed = new SetMaxSpeed(driveSS, LOW_SPEED);
        highSpeed = new SetMaxSpeed(driveSS, HIGH_SPEED);

        enabledHeadingLock = new EnableHeadingLock(driveSS, true);
        disableHeadingLock = new EnableHeadingLock(driveSS, false);

        forwardLock = new SetHeadingLock(driveSS, 90);
        leftLock = new SetHeadingLock(driveSS, 180);
        backLock = new SetHeadingLock(driveSS, -90);
        rightLock = new SetHeadingLock(driveSS, 0);

        //intake
        intakeIn = new SetIntakePower(intakeSS, INTAKE_POWER);
        intakeIdle = new SetIntakePower(intakeSS, 0);
        intakeOut = new SetIntakePower(intakeSS, -INTAKE_POWER);

        intakeUp = new SetIntakeAngle(intakeSS, INT_UP);
        intakeDown = new SetIntakeAngle(intakeSS, INT_DOWN);

        incrementIntakeUp = new IncrementIntakeAngle(intakeSS, INT_INCREMENT);
        incrementIntakeDown = new IncrementIntakeAngle(intakeSS, -INT_INCREMENT);

        variableIntakeAngle = new VariableIntakeAngle(
                intakeSS,
                () -> driver.getTouchX(),
                INT_UP,
                INT_DOWN
        );

        //elevator
        eleUp = new SetElevatorPower(eleSS, ELE_POWER);
        eleIdle = new SetElevatorPower(eleSS, 0);
        eleDown = new SetElevatorPower(eleSS, -ELE_POWER);

        eleTargetUp = new SetElevatorTarget(eleSS, ELE_UP);
        eleTargetMid = new SetElevatorTarget(eleSS, ELE_MID);
        eleTargetDown = new SetElevatorTarget(eleSS, ELE_DOWN);
        eleTargetHang = new SetElevatorTarget(eleSS, ELE_HANG);

        eleIncUp = new IncrementElevatorTarget(eleSS, ELE_INCREMENT);
        eleIncDown = new IncrementElevatorTarget(eleSS, -ELE_INCREMENT);

        //arm
        shoulderPosDeposit = new SetShoulderPosition(armSS, ARM_SHOULDER_DEPOSIT);
        shoulderPosPoised = new SetShoulderPosition(armSS, POISED_SHOULDER);
        shoulderPosGrab = new SetShoulderPosition(armSS, GRAB_SHOULDER);
        shoulderPosIdle = new SetShoulderPosition(armSS, ARM_SHOULDER_IDLE);

        shoulderIncUp = new IncrementShoulderPosition(armSS, 0.01);
        shoulderIncDown = new IncrementShoulderPosition(armSS, -0.01);

        wristPosDown = new SetWristPosition(armSS, ARM_WRIST_DEPOSIT);
        wristPosPoised = new SetWristPosition(armSS, POISED_WRIST);
        wristPosGrab = new SetWristPosition(armSS, GRAB_WRIST);
        wristPosUp = new SetWristPosition(armSS, ARM_WRIST_IDLE);

        wristIncDown = new IncrementWristPosition(armSS, 0.01);
        wristIncUp = new IncrementWristPosition(armSS, -0.01);

        pivotPosDown = new SetPivotPosition(armSS, ARM_PIVOT_DOWN);
        pivotPosMid = new SetPivotPosition(armSS, ARM_PIVOT_MID);
        pivotPosUp = new SetPivotPosition(armSS, ARM_PIVOT_UP);

        //grabber
        grabbersClosed = new SetGrabberPosition(grabSS, GRABBER_CLOSED);
        grabbersOpen = new SetGrabberPosition(grabSS, GRABBER_OPEN);

        grabberLeftClose = new SetLeftGrabberPosition(grabSS, GRABBER_CLOSED);
        grabberLeftOpen = new SetLeftGrabberPosition(grabSS, GRABBER_OPEN);
        grabberRightClose = new SetRightGrabberPosition(grabSS, GRABBER_CLOSED);
        grabberRightOpen = new SetRightGrabberPosition(grabSS, GRABBER_OPEN);

        //macros
        armInit = new SetArmPositions(
                armSS,
                GRAB_SHOULDER,
                GRAB_WRIST,
                ARM_PIVOT_MID
        );

        armDeposit = new SetEleArmPositions(
                eleSS,
                armSS,
                200,
                ARM_SHOULDER_DEPOSIT,
                ARM_WRIST_DEPOSIT,
                ARM_PIVOT_MID
        );

        armIdle = new SetEleArmPositions(
                eleSS,
                armSS,
                200,
                GRAB_SHOULDER,
                GRAB_WRIST,
                ARM_PIVOT_MID
        );

        armPoised = new SetEleArmPositions(
                eleSS,
                armSS,
                POISED_ELE,
                POISED_SHOULDER,
                POISED_WRIST,
                ARM_PIVOT_MID
        );

        armGrab = new SetEleArmPositions(
                eleSS,
                armSS,
                GRAB_ELE,
                GRAB_SHOULDER,
                GRAB_WRIST,
                ARM_PIVOT_MID
        );

        armBack = new SetEleArmPositions(
                eleSS,
                armSS,
                FLOOR_ELE,
                FLOOR_SHOULDER,
                FLOOR_WRIST,
                ARM_PIVOT_MID
        );

        armDown = new SetEleArmPositions(
                eleSS,
                armSS,
                0,
                GRAB_SHOULDER,
                GRAB_WRIST,
                ARM_PIVOT_DOWN
        );


        armDepositGroup = new SequentialCommandGroup(
                new SetArmPositions(
                        armSS,
                        ARM_SHOULDER_DEPOSIT,
                        ARM_WRIST_DEPOSIT,
                        ARM_PIVOT_MID
                ),
                new SetWristPosition(armSS, ARM_WRIST_TEST),
                new WaitCommand(100),
                new SetWristPosition(armSS, ARM_WRIST_DEPOSIT)
        );

        armIdleGroup = new SequentialCommandGroup(
                new SetShoulderPosition(armSS, ARM_SHOULDER_IDLE),
                new WaitCommand(50),
                new SetWristPosition(armSS, ARM_WRIST_IDLE),
                new WaitCommand(50),
                new SetShoulderPosition(armSS, GRAB_SHOULDER)
        );


        autoGrab = new ConditionalCommand(new SetGrabberPosition(grabSS, GRABBER_CLOSED), new SetGrabberPosition(grabSS, GRABBER_OPEN), () -> intakeSS.isLoaded());

        //gamepads
        driver = new PoofyGamepadEx(gamepad1);
        operator = new PoofyGamepadEx(gamepad2);

        //telemetry
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //loops
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
            tad("Front Left Current", driveSS.getfLCurrent());
            tad("Front Right Current", driveSS.getfRCurrent());
            tad("Rear Left Current", driveSS.getrLCurrent());
            tad("Rear Right Current", driveSS.getrRCurrent());
            tal();
        }

        if (DEBUG_INTAKE) {
            tal("=== INTAKE DEBUG INFO ===");
            tad("Intake Power", intakeSS.getPower());
            tad("Intake Position", intakeSS.getServoPosition());
            tad("Intake Motor Position", intakeSS.getIntakePosition());
            tad("Intake Mod Position", intakeSS.getModPosition());
            tad("Intake Target", intakeSS.getTarget());
            tad("Intake Back Pixel Loaded", intakeSS.isBackPixelLoaded());
            tad("Intake Front Pixel Loaded", intakeSS.isFrontPixelLoaded());
            tad("Intake Left Motor Current", intakeSS.getlCurrent());
            tad("Intake Right Motor Current", intakeSS.getrCurrent());
            tal();
        }

        if (DEBUG_ELEVATOR) {
            tal("=== ELEVATOR DEBUG INFO ===");
            tad("Elevator Power", eleSS.getPower());
            tad("Elevator Position", eleSS.getPosition());
            tad("Elevator Left Current", eleSS.getLeftCurrent());
            tad("Elevator Right Current", eleSS.getRightCurrent());
            tad("Elevator Target", eleSS.getTarget());
            tal();
        }

        if (DEBUG_ARM) {
            tal("=== ARM DEBUG INFO ===");
            tad("Arm Shoulder Position", armSS.getShoulderPosition());
            tad("Arm Wrist Position", armSS.getWristPosition());
            tad("Arm Pivot Position", armSS.getPivotPosition());
            tal();
        }

        if (DEBUG_GRABBER) {
            tal("=== GRABBER DEBUG INFO");
            tad("Grabber Position", grabSS.getLeftPos());
            tal();
        }

        if (DEBUG_VISION) {
            tal("=== VISION DEBUG INFO ===");
            tad("Tag Pose", driveSS.getTagLocalizer().getTagPose());
            tad("Tag Readings", driveSS.getTagLocalizer().getTagReadings());
            tad("Camera Pose", driveSS.getTagLocalizer().getCameraPose());
            tad("Robot Pose", driveSS.getRobotPose());
        }

        if (!auto) {
            robot.read(driveSS, intakeSS, eleSS, armSS, grabSS);

            robot.loop(driveSS, intakeSS, eleSS, armSS, grabSS);

            robot.write(driveSS, intakeSS, eleSS, armSS, grabSS);
        }
    }


    protected void tau() {
        telemetry.update();
    }
    protected void tal() {
        telemetry.addLine();
    }
    protected void tal(String caption) {
        telemetry.addLine(caption);
    }
    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }


    protected GamepadButton gp1(GamepadKeys.Button button) {
        return driver.getGamepadButton(button);
    }

    protected GamepadTrigger gp1(GamepadKeys.Trigger trigger) {
        return driver.getGamepadTrigger(trigger);
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
            return driver.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return driver.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return driver.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return driver.getGamepadTrigger(trigger);
        }
    }



    protected GamepadButton gp2(GamepadKeys.Button button) {
        return operator.getGamepadButton(button);
    }

    protected GamepadTrigger gp2(GamepadKeys.Trigger trigger) {
        return operator.getGamepadTrigger(trigger);
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
            return operator.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return operator.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return operator.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return operator.getGamepadTrigger(trigger);
        }
    }

    @Override
    public void initLoop() {

    }

    @Override
    public void runOnce() {

    }
}
