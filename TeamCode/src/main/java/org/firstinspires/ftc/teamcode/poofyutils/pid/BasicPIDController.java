package org.firstinspires.ftc.teamcode.poofyutils.pid;

public class BasicPIDController {

    //variable declaration
    private final double kP;
    private final double kI;
    private final double kD;

    private double previousError;
    private double totalError;
    private double velocityError;

    private double lastTimeStamp;

    //constructor
    public BasicPIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    //the calculate method. returns the motor power based off pid
    public double calculate(double measuredPosition, double targetPosition) {
        //calculates the period - the time since it was last updated
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        double period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        //calculates position error
        double positionError = targetPosition - measuredPosition;
        //calculates integral
        totalError += ((positionError + previousError) / 2) * period;
        //calculates derivative - fun fact, it is also the velocity error
        velocityError = (positionError - previousError) / period;
        previousError = positionError;

        //returns the desired power using kP, kI, and kD
        return positionError * kP
                + totalError * kI
                + velocityError * kD;
    }

}

