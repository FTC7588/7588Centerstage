package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;

public class DrivetrainSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private final MecanumDrive drive;

    private double heading;

    public DrivetrainSubsystem(RobotHardware robot) {
        this.robot = robot;

        drive = new MecanumDrive(
                robot.fL,
                robot.fR,
                robot.rL,
                robot.rR
        );

    }

    public void read() {

    }

    public void loop() {
        heading = robot.getHeading();
    }

    public void write() {
        drive.write();
    }

    public double getHeading() {
        return Math.toDegrees(heading);
    }

    public void robotCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean pidTurning) {
        if (!pidTurning) {
            drive.driveRobotCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed
            );
        } else {
            drive.driveRobotCentricPID(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    heading
            );
        }
//        mode = DriveMode.ROBOT_CENTRIC;
    }

    public void fieldCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean pidTurning) {
        if (!pidTurning) {
            drive.driveFieldCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    Math.toDegrees(robot.getHeading())
            );
        } else {
            drive.driveFieldCentricPID(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    heading
            );
        }
//        mode = DriveMode.FIELD_CENTRIC;
    }

}
