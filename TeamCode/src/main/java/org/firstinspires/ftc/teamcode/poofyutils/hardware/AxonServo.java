package org.firstinspires.ftc.teamcode.poofyutils.hardware;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AxonServo {

    private final CRServo servo;
    private final AnalogInput potentiometer;

    public AxonServo(HardwareMap hwMap, String servoName, String sensorName) {
        servo = new CRServo(hwMap, servoName);
        potentiometer = hwMap.get(AnalogInput.class, sensorName);
    }

//    public void setPosition(double pos) {
//        servo.setPosition(pos);
//    }
//
//    public void setPWMRange(PwmControl.PwmRange range) {
//        servo.setPwmRange(range);
//    }
//
//    public void setDirection(Servo.Direction direction) {
//        servo.setDirection(direction);
//    }

    public double getAngle() {
        return potentiometer.getVoltage() / 3.3 * 360;
    }

}
