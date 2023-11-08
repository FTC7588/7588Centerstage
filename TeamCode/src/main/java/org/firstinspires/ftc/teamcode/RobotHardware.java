package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.geometry.EulerAngles;

import java.util.List;

public class RobotHardware {

    //drive
    public DcMotorEx fL, fR, rL, rR;

    //intake
    public DcMotorEx intLMotor, intRMotor;
    public ServoImplEx intLServo, intRServo;

    //arm
    public DcMotorEx eleL, eleR;
    public ServoImplEx armL, armR, armWrist, armPivot;

    //grabber
    public ServoImplEx grab1, grab2;

    //imu
    public IMU imu;
    public double rollOffset, pitchOffset, headingOffset;

    public CameraName C920;

    //angles
    public EulerAngles angles;

    //instantiation code
    private HardwareMap hwMap;

    private List<LynxModule> hubs;

    public boolean enabled;

    private static RobotHardware instance = null;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //hardware
        imu = hwMap.get(IMU.class, "imu");

        fL = hwMap.get(DcMotorEx.class, "fL");
        fR = hwMap.get(DcMotorEx.class, "fR");
        rL = hwMap.get(DcMotorEx.class, "rL");
        rR = hwMap.get(DcMotorEx.class, "rR");

        intLMotor = hwMap.get(DcMotorEx.class, "intL");
        intRMotor = hwMap.get(DcMotorEx.class, "intR");
        intLServo = hwMap.get(ServoImplEx.class, "intLS");
        intRServo = hwMap.get(ServoImplEx.class, "intRS");

        eleL = hwMap.get(DcMotorEx.class, "eleL");
        eleR = hwMap.get(DcMotorEx.class, "eleR");

        armL = hwMap.get(ServoImplEx.class, "armL");
        armR = hwMap.get(ServoImplEx.class, "armR");
        armWrist = hwMap.get(ServoImplEx.class, "armWrist");
        armPivot = hwMap.get(ServoImplEx.class, "armPivot");

        grab1 = hwMap.get(ServoImplEx.class, "grab1");
        grab2 = hwMap.get(ServoImplEx.class, "grab2");

        C920 = hwMap.get(WebcamName.class, "Webcam 1");

        //imu
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        //drive
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        rL.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intake
        intLMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intRServo.setDirection(Servo.Direction.REVERSE);

        //elevator
        eleL.setDirection(DcMotorSimple.Direction.FORWARD);
        eleR.setDirection(DcMotorSimple.Direction.REVERSE);
        eleL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eleR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eleL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eleR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //arm
        armPivot.setDirection(Servo.Direction.REVERSE);
        armL.setDirection(Servo.Direction.FORWARD);
        armR.setDirection(Servo.Direction.REVERSE);

        //grabber
        grab2.setDirection(Servo.Direction.REVERSE);

        angles = new EulerAngles(
                imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS),
                imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );
    }

    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public void read(DrivetrainSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elevator, ArmSubsystem arm) {
        angles.yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        drive.read();
        intake.read();
        elevator.read();
        arm.read();
    }

    public void loop(DrivetrainSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elevator, ArmSubsystem arm) {
        drive.loop();
        intake.loop();
        elevator.loop();
        arm.loop();
    }

    public void write(DrivetrainSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elevator, ArmSubsystem arm) {
        drive.write();
        intake.write();
        elevator.write();
        arm.write();
    }

    public double getHeading() {
        return angles.yaw - headingOffset;
    }

    public void resetIMU() {
        rollOffset = angles.roll;
        pitchOffset = angles.pitch;
        headingOffset = angles.yaw;
    }



}
