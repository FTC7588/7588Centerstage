package org.firstinspires.ftc.teamcode.poofyutils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.poofyutils.pid.PoofyPIDCoefficients;
import org.firstinspires.ftc.teamcode.poofyutils.pid.PoofyPIDController;

public class MecanumDrive {

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private double frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed;
    private double maxOutput = 1;

    private final PoofyPIDController xController;
    private final PoofyPIDController yController;
    private final PoofyPIDController thetaController;
    private double maxTurnSpeedPID;

    private double turnSpeed;
    private double theta;

    private double forwardMax, backMax, leftMax, rightMax;

    private boolean headingLock;
    private double headingLockTarget;

    private Pose2d currentPose;

    public MecanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        xController = new PoofyPIDController.Builder().build();
        yController = new PoofyPIDController.Builder().build();
        thetaController = new PoofyPIDController.Builder().build();
        currentPose = new Pose2d(0, 0, 0);

        forwardMax = backMax = leftMax = rightMax = 1;
    }

    public MecanumDrive(DcMotorEx frontLeft,
                        DcMotorEx frontRight,
                        DcMotorEx backLeft,
                        DcMotorEx backRight,
                        PoofyPIDCoefficients xCoeffs,
                        PoofyPIDCoefficients yCoeffs,
                        PoofyPIDCoefficients thetaCoeffs,
                        double maxTurnSpeedPID
    ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        xController = new PoofyPIDController(xCoeffs);
        yController = new PoofyPIDController(yCoeffs);

        thetaController = new PoofyPIDController(thetaCoeffs);
        thetaController.setInputBounds(-180, 180);
        thetaController.setOutputBounds(-1, 1);

        this.maxTurnSpeedPID = maxTurnSpeedPID;

        currentPose = new Pose2d(0, 0, 0);

        forwardMax = backMax = leftMax = rightMax = 1;
    }



    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        double denominator = Math.max(Math.abs(forwardSpeed) + Math.abs(strafeSpeed) + Math.abs(turnSpeed), 1);
        double frontLeftPower = (forwardSpeed + strafeSpeed + turnSpeed) / denominator;
        double backLeftPower = (forwardSpeed - strafeSpeed + turnSpeed) / denominator;
        double frontRightPower = (forwardSpeed - strafeSpeed - turnSpeed) / denominator;
        double backRightPower = (forwardSpeed + strafeSpeed - turnSpeed) / denominator;

        frontLeftSpeed = frontLeftPower;
        backLeftSpeed = backLeftPower;
        frontRightSpeed = frontRightPower;
        backRightSpeed = backRightPower;
    }

    public void driveRobotCentricLock(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {
        if (headingLock) {
            turnSpeed = thetaController.calculate(gyroAngle);
        }
        driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void driveRobotCentricPID(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {
//        thetaTarget -= turnSpeed * maxTurnSpeedPID;
//        turnSpeed = -thetaController.calculate(gyroAngle, Math.toRadians(thetaTarget));

        driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void driveFieldCentric(double strafeSpeed,
                                  double forwardSpeed,
                                  double turnSpeed,
                                  double gyroAngle
    ) {
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

//        if (input.getX() >= 0) {
//            input = new Vector2d(Math.min(input.getX(), rightMax), input.getY());
//        }

//        input = input.getX() >= 0 ? new Vector2d(Math.min(input.getX(), rightMax), input.getY()) : new Vector2d(Math.max(input.getX(), leftMax), input.getY());
//
//        input = input.getY() >= 0 ? new Vector2d(input.getX(), Math.min(input.getY(), forwardMax)) : new Vector2d(input.getX(), Math.max(input.getY(), backMax));

        driveRobotCentric(
                input.getX(),
                input.getY(),
                turnSpeed
        );
    }

    public void driveFieldCentricPID(double strafeSpeed,
                                  double forwardSpeed,
                                  double turnSpeed,
                                  double gyroAngle
    ) {
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        driveRobotCentricPID(
                input.getX(),
                input.getY(),
                turnSpeed,
                gyroAngle
        );
    }

    public void drivePointCentric(double strafeSpeed,
                                  double forwardSpeed,
                                  double gyroAngle,
                                  Vector2d target,
                                  Pose2d currentPose,
                                  double angleOffset
    ) {

        this.currentPose = currentPose;

        Vector2d difference = target.subtract(currentPose.getVector());

        theta = difference.getAngle() + Math.toRadians(angleOffset);

        turnSpeed = thetaController.calculate(theta, currentPose.getTheta());

        double turnError = Math.abs(theta = currentPose.getTheta());
//
//        strafeSpeed *= Math.cos(Range.clip(turnError, -Math.PI/2, Math.PI/2));
//        forwardSpeed *= Math.cos(Range.clip(turnError, -Math.PI/2, Math.PI/2));

        driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                -turnSpeed,
                gyroAngle
        );
    }

    public void driveFollowTag(Pose2d tagPose,
                               Pose2d targetFollowingPose
    ) {
        xController.setTargetPosition(targetFollowingPose.getY());
        double strafeSpeed = xController.calculate(tagPose.getY());

        yController.setTargetPosition(targetFollowingPose.getX());
        double forwardSpeed = yController.calculate(tagPose.getX());

        double turnSpeed = thetaController.calculate(Math.toRadians(targetFollowingPose.getTheta()), Math.toRadians(tagPose.getTheta()));

        driveRobotCentric(
                strafeSpeed,
                -forwardSpeed,
                turnSpeed
        );
    }



    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public double getTurnSpeed() {
        return turnSpeed;
    }

    public double getMaxSpeed() {
        return maxOutput;
    }

    public void setMaxSpeed(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public void setHeadingLock(boolean enabled) {
        headingLock = enabled;
    }

    public void setHeadingLockTarget(double target) {
        headingLockTarget = target;
        thetaController.setTargetPosition(target);
    }

    public boolean getHeadingLock() {
        return headingLock;
    }

    public double getHeadingLockTarget() {
        return headingLockTarget;
    }


    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed,
                                     double backLeftSpeed, double backRightSpeed) {
        this.frontLeft
                .setPower(frontLeftSpeed * maxOutput);
        this.frontRight
                .setPower(frontRightSpeed * maxOutput);
        this.backLeft
                .setPower(backLeftSpeed * maxOutput);
        this.backRight
                .setPower(backRightSpeed * maxOutput);
    }

    public void write() {
        driveWithMotorPowers(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

}
