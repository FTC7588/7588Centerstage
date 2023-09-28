package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.geometry.EulerAngles;

import java.util.List;

public class RobotHardware {

    //drive
    public DcMotorEx fL, fR, rL, rR;

    //imu
    public IMU imu;

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

        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        //drive
        fL = hwMap.get(DcMotorEx.class, "fL");
        fR = hwMap.get(DcMotorEx.class, "fR");
        rL = hwMap.get(DcMotorEx.class, "rL");
        rR = hwMap.get(DcMotorEx.class, "rR");

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

    public void read(DrivetrainSubsystem drive) {
        angles.yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        drive.read();
    }

    public void loop(DrivetrainSubsystem drive) {
        drive.loop();
    }

    public void write(DrivetrainSubsystem drive) {
        drive.write();
    }

    public double getHeading() {
        return angles.yaw;
    }



}
