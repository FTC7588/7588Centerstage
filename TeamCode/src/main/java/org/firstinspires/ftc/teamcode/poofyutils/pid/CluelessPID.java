package org.firstinspires.ftc.teamcode.poofyutils.pid;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

// https://www.ctrlaltftc.com/the-pid-controller
@Config
public class CluelessPID {
    public double p;
    public double i;
    public double d;
    public CluelessPID(double P, double I, double D){
        p=P;
        i=I;
        d=D;
    }
    double integral = 0;
    long lastLoopTime = System.nanoTime();
    double lastError = 0;
    double lastPower = 0;
    int counter = 0;
    double loopTime = 0.0;

    public void resetIntegral() {
        integral = 0;
    }

    public double update(double error, double min, double max){
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10000000;
        }
        counter ++;

        if (System.nanoTime()-lastLoopTime < 30e6){ //20 ms for example
            return lastPower;
        }

        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime)/1000000000.0;
        lastLoopTime = currentTime; // lastLoopTime's start time

        double proportion = p * error;
        integral += error * i * loopTime;
        double derivative = d * (error - lastError)/loopTime;

        lastError = error;
        lastPower = minMaxClip(proportion + integral + derivative, min, max);
        return lastPower;
    }

    public double minMaxClip(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
    }

    public void updatePID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}