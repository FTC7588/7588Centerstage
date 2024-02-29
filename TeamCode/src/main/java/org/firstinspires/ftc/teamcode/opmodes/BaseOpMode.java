package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.commands.arm.SetPivotPositions;
import org.firstinspires.ftc.teamcode.commands.arm.SetShoulderPosition;
import org.firstinspires.ftc.teamcode.commands.arm.SetWristPosition;
import org.firstinspires.ftc.teamcode.commands.drive.EnableHeadingLock;
import org.firstinspires.ftc.teamcode.commands.drive.FieldCentric;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTag;
import org.firstinspires.ftc.teamcode.commands.drive.RobotCentric;
import org.firstinspires.ftc.teamcode.commands.drive.SetDronePosition;
import org.firstinspires.ftc.teamcode.commands.drive.SetHeadingLock;
import org.firstinspires.ftc.teamcode.commands.drive.SetMaxSpeed;
import org.firstinspires.ftc.teamcode.commands.elevator.IncrementElevatorOffset;
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
import org.firstinspires.ftc.teamcode.poofyutils.processors.Alliance;
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
import static org.firstinspires.ftc.teamcode.RobotHardware.USING_TAGS;

import java.util.function.BooleanSupplier;

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

    protected SetMaxSpeed lowSpeed;
    protected SetMaxSpeed highSpeed;

    protected EnableHeadingLock enabledHeadingLock;
    protected EnableHeadingLock disableHeadingLock;

    protected SetHeadingLock forwardLock;
    protected SetHeadingLock leftLock;
    protected SetHeadingLock backLock;
    protected SetHeadingLock rightLock;

    protected InstantCommand resetIMU;

    protected SetDronePosition droneRelease;
    protected SetDronePosition droneHold;

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

    protected IncrementElevatorOffset eleIncOffsetUp;
    protected IncrementElevatorOffset eleIncOffsetDown;

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
    protected SetPivotPosition pivotPosLeftDiag;
    protected SetPivotPosition pivotPosMid;
    protected SetPivotPosition pivotPosRightDiag;
    protected SetPivotPosition pivotPosUp;

    protected SetPivotPosition pivotPosNormLeft;
    protected SetPivotPosition pivotPosNormDown;
    protected SetPivotPosition pivotPosNormRight;
    protected SetPivotPosition pivotPosNormUp;
    protected SetPivotPosition pivotPosRotLeft;
    protected SetPivotPosition pivotPosRotDown;
    protected SetPivotPosition pivotPosRotRight;

    protected SetPivotPosition pivotPosDownDefalt;
    protected SetPivotPosition pivotPosLeftDefault;
    protected SetPivotPosition pivotPosRightDefault;

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
    protected SequentialCommandGroup armPoisedGroup;
    protected SequentialCommandGroup armGrabGroup;
    protected SequentialCommandGroup armBackGroup;
    protected SequentialCommandGroup autoArmBack;
    protected SequentialCommandGroup armIdleGroup;

    protected ConditionalCommand autoGrab;

//    protected SetPivotPositions pivot

    protected PoofyGamepadEx driver;
    protected PoofyGamepadEx operator;

    protected MultipleTelemetry tele;

    protected double loopTime;
    protected MovingAverage loopAvg;

    protected boolean auto = false;
    protected Alliance alliance;

//    protected PivotRotatedState pivotState = PivotRotatedState.NORMAL;
    protected ArmState armState = ArmState.EXTENDED;

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

        resetIMU = new InstantCommand(robot::resetIMU);

        droneHold = new SetDronePosition(driveSS, DRONE_HOLD);
        droneRelease = new SetDronePosition(driveSS, DRONE_RELEASE);

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
        eleUp = new SetElevatorPower(eleSS, -ELE_POWER);
        eleIdle = new SetElevatorPower(eleSS, 0);
        eleDown = new SetElevatorPower(eleSS, ELE_POWER);

        eleTargetUp = new SetElevatorTarget(eleSS, ELE_UP);
        eleTargetMid = new SetElevatorTarget(eleSS, ELE_MID);
        eleTargetDown = new SetElevatorTarget(eleSS, ELE_DOWN);
        eleTargetHang = new SetElevatorTarget(eleSS, ELE_HANG);

        eleIncUp = new IncrementElevatorTarget(eleSS, ELE_INCREMENT);
        eleIncDown = new IncrementElevatorTarget(eleSS, -ELE_INCREMENT);

        eleIncOffsetUp = new IncrementElevatorOffset(eleSS, ELE_INCREMENT);
        eleIncOffsetDown = new IncrementElevatorOffset(eleSS, -ELE_INCREMENT);

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
        pivotPosLeftDiag = new SetPivotPosition(armSS, ARM_PIVOT_UP_MID);
        pivotPosMid = new SetPivotPosition(armSS, ARM_PIVOT_MID);
        pivotPosRightDiag = new SetPivotPosition(armSS, ARM_PIVOT_DOWN_MID);
        pivotPosUp = new SetPivotPosition(armSS, ARM_PIVOT_UP);

        pivotPosNormLeft = new SetPivotPosition(armSS, ARM_PIVOT_NORM_LEFT);
        pivotPosNormDown = new SetPivotPosition(armSS, ARM_PIVOT_NORM_DOWN);
        pivotPosNormRight = new SetPivotPosition(armSS, ARM_PIVOT_NORM_RIGHT);

        pivotPosNormUp = new SetPivotPosition(armSS, ARM_PIVOT_NORM_UP);

        pivotPosRotLeft = new SetPivotPosition(armSS, ARM_PIVOT_ROT_LEFT);
        pivotPosRotDown = new SetPivotPosition(armSS, ARM_PIVOT_ROT_DOWN);
        pivotPosRotRight = new SetPivotPosition(armSS, ARM_PIVOT_ROT_RIGHT);

        pivotPosDownDefalt = pivotPosNormDown;
        pivotPosLeftDefault = pivotPosNormLeft;
        pivotPosRightDefault = pivotPosNormRight;

        //grabber
        grabbersClosed = new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED);
        grabbersOpen = new SetGrabberPosition(grabSS, GRABBER_ONE_OPEN, GRABBER_TWO_OPEN);

        grabberLeftClose = new SetLeftGrabberPosition(grabSS, GRABBER_ONE_CLOSED);
        grabberLeftOpen = new SetLeftGrabberPosition(grabSS, GRABBER_ONE_OPEN);
        grabberRightClose = new SetRightGrabberPosition(grabSS, GRABBER_TWO_CLOSED);
        grabberRightOpen = new SetRightGrabberPosition(grabSS, GRABBER_TWO_OPEN);

        //macros
        armIdleGroup = new SequentialCommandGroup(
                new SetArmPositions(
                        armSS,
                        GRAB_SHOULDER,
                        GRAB_WRIST,
                        ARM_PIVOT_MID
                ),
                new InstantCommand(() -> {
                    armSS.pivotRotatedState = ArmSubsystem.PivotRotatedState.NORMAL;
                    armSS.pivotPositionState = ArmSubsystem.PivotPositionState.UP;
                }),
                new SetShoulderPosition(armSS, ARM_SHOULDER_IDLE),
                new WaitCommand(100),
                new SetShoulderPosition(armSS, GRAB_SHOULDER)
//                new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED)
        );

        armBackGroup = new SequentialCommandGroup(
                new SetArmPositions(
                        armSS,
                        FLOOR_SHOULDER,
                        FLOOR_WRIST,
                        ARM_PIVOT_MID
                ),
                new SetWristPosition(armSS, ARM_WRIST_TEST),
                new WaitCommand(100),
                new SetWristPosition(armSS, FLOOR_WRIST)
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
                new SetWristPosition(armSS, ARM_WRIST_DEPOSIT),
                new WaitCommand(100),
                new InstantCommand(() -> armSS.pivotPositionState = ArmSubsystem.PivotPositionState.MID)
        );

        autoArmBack = new SequentialCommandGroup(
                new SetArmPositions(
                        armSS,
                        ARM_AUTO,
                        0,
                        ARM_PIVOT_NORM_DOWN
                ),
                new SetWristPosition(armSS, ARM_WRIST_TEST),
                new WaitCommand(100),
                new SetWristPosition(armSS, FLOOR_WRIST),
                new WaitCommand(100),
                new InstantCommand(() -> armSS.pivotPositionState = ArmSubsystem.PivotPositionState.MID)
        );

        armGrabGroup = new SequentialCommandGroup(
                new SetShoulderPosition(armSS, ARM_SHOULDER_IDLE),
                new WaitCommand(75),
                new SetWristPosition(armSS, ARM_WRIST_IDLE),
                new WaitCommand(75),
                new SetShoulderPosition(armSS, GRAB_SHOULDER)
        );

        armPoisedGroup = new SequentialCommandGroup(
                new SetShoulderPosition(armSS, POISED_SHOULDER),
                new SetWristPosition(armSS, POISED_WRIST)
        );


        autoGrab = new ConditionalCommand(new SetGrabberPosition(grabSS, GRABBER_ONE_CLOSED, GRABBER_TWO_CLOSED), new SetGrabberPosition(grabSS, GRABBER_ONE_OPEN, GRABBER_TWO_OPEN), () -> true);

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
            tad("Front Left Power", driveSS.drive.getFrontLeftSpeed());
            tad("Front Right Power", driveSS.drive.getFrontRightSpeed());
            tad("Rear Left Current", driveSS.drive.getBackLeftSpeed());
            tad("Rear Right Current", driveSS.drive.getBackRightSpeed());
            tal();
        }

        if (DEBUG_INTAKE) {
            tal("=== INTAKE DEBUG INFO ===");
            tad("Intake Power", intakeSS.getPower());
            tad("Intake Position", intakeSS.getServoPosition());
            tad("Intake Motor Position", intakeSS.getIntakePosition());
            tad("Intake Mod Position", intakeSS.getModPosition());
            tad("Intake Target", intakeSS.getTarget());
            if (RobotHardware.USING_SENSORS) {

            }
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

        if (DEBUG_VISION && USING_TAGS) {
            tal("=== VISION DEBUG INFO ===");
            tad("Tag Pose", driveSS.getTagLocalizer().getTagPose());
            tad("Tag Readings", driveSS.getTagLocalizer().getTagReadings());
            tad("Camera Pose", driveSS.getTagLocalizer().getCameraPose());
            tad("Robot Pose", driveSS.getTagPose());
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

    protected Trigger gp2(GamepadKeys.Button button, BooleanSupplier state) {
        if (state.getAsBoolean()) {
            return operator.getGamepadButton(button);
        } else {
            return new Trigger();
//            return operator.getGamepadTrigger(GamepadKeys.Trigger.LEFT_TRIGGER).and(gp2(GamepadKeys.Trigger.LEFT_TRIGGER).negate());
        }
    }

    protected Trigger gp2(GamepadKeys.Trigger trigger, BooleanSupplier state) {
        if (state.getAsBoolean()) {
            return operator.getGamepadTrigger(trigger);
        } else {
            return new Trigger();
        }
    }

    protected Trigger gp2(GamepadKeys.Button button, int layer) {
        if (layer == 1) {
            return operator.getGamepadButton(button)
                    .and(gp2(Constants.CONTROL_LAYER_2).negate())
                    .and(gp2(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return operator.getGamepadButton(button)
                    .and(gp2(Constants.CONTROL_LAYER_2))
                    .and(gp2(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return operator.getGamepadButton(button)
                    .and(gp2(Constants.CONTROL_LAYER_2).negate())
                    .and(gp2(Constants.CONTROL_LAYER_3));
        } else {
            return operator.getGamepadButton(button);
        }
    }

    protected Trigger gp2(GamepadKeys.Trigger trigger, int layer) {
        if (layer == 1) {
            return operator.getGamepadTrigger(trigger)
                    .and(gp2(Constants.CONTROL_LAYER_2).negate())
                    .and(gp2(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return operator.getGamepadTrigger(trigger)
                    .and(gp2(Constants.CONTROL_LAYER_2))
                    .and(gp2(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return operator.getGamepadTrigger(trigger)
                    .and(gp2(Constants.CONTROL_LAYER_2).negate())
                    .and(gp2(Constants.CONTROL_LAYER_3));
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

    protected enum ArmState {
        GRABBED,
        RETRACTED,
        EXTENDED
    }



}
