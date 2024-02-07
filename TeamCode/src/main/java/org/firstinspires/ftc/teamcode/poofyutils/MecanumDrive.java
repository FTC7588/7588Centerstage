package org.firstinspires.ftc.teamcode.poofyutils;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.poofyutils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.poofyutils.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.poofyutils.pid.CluelessPID;
import org.firstinspires.ftc.teamcode.poofyutils.pid.PoofyPIDCoefficients;
import org.firstinspires.ftc.teamcode.poofyutils.pid.PoofyPIDController;

public class MecanumDrive {

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private double frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed;
    private double maxOutput = 1;

    public final CluelessPID xController;
    public final CluelessPID yController;
    public final PoofyPIDController thetaController;

    private double turnSpeed;
    private double theta;

    private double forwardMax, backMax, leftMax, rightMax;

    private boolean headingLock;
    private double headingLockTarget;

    private Pose2d currentPose;
    private Pose2d targetPose = new Pose2d(0, 0, 0);

    public MecanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        xController = new CluelessPID(0, 0, 0);
        yController = new CluelessPID(0, 0, 0);
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
                        PoofyPIDCoefficients thetaCoeffs
    ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        xController = new CluelessPID(xCoeffs.kP, xCoeffs.kI, 50);
        yController = new CluelessPID(yCoeffs.kP, yCoeffs.kI, 50);

        thetaController = new PoofyPIDController(thetaCoeffs);
        thetaController.setInputBounds(-Math.PI, Math.PI);
        thetaController.setOutputBounds(-1, 1);

        currentPose = new Pose2d(0, 0, 0);

        forwardMax = backMax = leftMax = rightMax = 1;
    }



    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        strafeSpeed *= 1.8;
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

    public void driveRobotCentric(
            double strafeSpeed,
            double forwardSpeed,
            double turnSpeed,
            double fLStatic,
            double fRStatic,
            double rLStatic,
            double rRStatic) {
        strafeSpeed *= 1.8;
        double denominator = Math.max(Math.abs(forwardSpeed) + Math.abs(strafeSpeed) + Math.abs(turnSpeed), 1);
        double frontLeftPower = (forwardSpeed + strafeSpeed + turnSpeed) / denominator;
        double backLeftPower = (forwardSpeed - strafeSpeed + turnSpeed) / denominator;
        double frontRightPower = (forwardSpeed - strafeSpeed - turnSpeed) / denominator;
        double backRightPower = (forwardSpeed + strafeSpeed - turnSpeed) / denominator;

        frontLeftSpeed = staticConverter(frontLeftPower, fLStatic);
        backLeftSpeed = staticConverter(backLeftPower, rLStatic);
        frontRightSpeed = staticConverter(frontRightPower, fRStatic);
        backRightSpeed = staticConverter(backRightPower, rRStatic);
    }

    public double staticConverter(double power, double staticValue) {
        if (power > 0 ) {
            return power + staticValue;
        } else if (power < 0) {
            return power - staticValue;
        } else {
            return 0;
        }
    }

    public void driveRobotCentricLock(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {
        if (headingLock) {
            turnSpeed = thetaController.calculate(gyroAngle);
        }
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

    public void driveFieldCentric(double strafeSpeed,
                                  double forwardSpeed,
                                  double turnSpeed,
                                  double gyroAngle,
                                  double fL,
                                  double fR,
                                  double rL,
                                  double rR
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
                turnSpeed,
                fL,
                fR,
                rL,
                rR
        );
    }

    public void driveFollowTag(Pose2d tagPose,
                               Pose2d targetFollowingPose
    ) {
//        xController.setSetPoint(targetFollowingPose.getY());
//        double strafeSpeed = xController.calculate(tagPose.getY());
//
//        yController.setSetPoint(targetFollowingPose.getX());
//        double forwardSpeed = yController.calculate(tagPose.getX());

        double turnSpeed = thetaController.calculate(Math.toRadians(targetFollowingPose.getTheta()), Math.toRadians(tagPose.getTheta()));

//        driveRobotCentric(
//                strafeSpeed,
//                -forwardSpeed,
//                turnSpeed
//        );
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
        thetaController.setTargetPosition(Math.toRadians(target));
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

    public double getBackRightSpeed() {
        return backRightSpeed;
    }

    public double getFrontLeftSpeed() {
        return frontLeftSpeed;
    }

    public double getFrontRightSpeed() {
        return frontRightSpeed;
    }

    public double getBackLeftSpeed() {
        return backLeftSpeed;
    }

    //pidtopoint stuff

    public boolean reachedTarget(double tolerance, Pose2d currentPose) {
        return Math.sqrt((targetPose.getX() - currentPose.getX())*(targetPose.getX() - currentPose.getX()) + (targetPose.getY() - currentPose.getY())*(targetPose.getY() - currentPose.getY())) <= tolerance;
    }

    public boolean reachedXTarget(double tolerance, Pose2d currentPose) {
        return Math.abs(targetPose.getX() - currentPose.getX()) < tolerance;
    }

    public boolean reachedYTarget(double tolerance, Pose2d currentPose) {
        return Math.abs(targetPose.getY() - currentPose.getY()) < tolerance;
    }

    public boolean reachedThetaTarget(double tolerance, Pose2d currentPose) {
        return Math.abs(Math.toDegrees(targetPose.getTheta() - currentPose.getTheta())) < tolerance;
    }

    public boolean reachedHeading(double tolerance, Pose2d currentPose) {
        double diff = targetPose.getTheta() - currentPose.getTheta();
        while(diff>Math.PI) diff -= Math.PI * 2.0;
        while(diff<-Math.PI) diff += Math.PI * 2.0;
        return Math.abs(diff) <= tolerance;
    }

    public void driveFollowPose(Pose2d targetPose, Pose2d currentPose, double gyroAngle, double posTol, double headingTolDeg, double fL, double fR, double rL, double rR) {
        this.targetPose = targetPose;
        double forwardSpeed, strafeSpeed, turnSpeed;

//        xController.setSetPoint(targetPose.getX());

        forwardSpeed = reachedXTarget(posTol, currentPose) ? 0 : xController.update(targetPose.getX() - currentPose.x, -1, 1);
//        forwardSpeed = xController.calculate(currentPose.getX());

//        yController.setSetPoint(targetPose.getY());

        strafeSpeed = reachedYTarget(posTol, currentPose) ? 0 :  yController.update(targetPose.getY() - currentPose.y, -1, 1);
//        strafeSpeed = yController.calculate(currentPose.getY());

        thetaController.setTargetPosition(targetPose.getTheta());

        turnSpeed = reachedHeading(Math.toRadians(headingTolDeg), currentPose) ? 0 : thetaController.calculate(currentPose.getTheta());
//        turnSpeed = thetaController.calculate(currentPose.getTheta());

        driveFieldCentric(
                -strafeSpeed,
                forwardSpeed,
                -turnSpeed,
                currentPose.getTheta(),
                fL,
                fR,
                rL,
                rR
        );
    }

}
