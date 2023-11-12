package org.firstinspires.ftc.teamcode.poofyutils.hardware;

import org.firstinspires.ftc.teamcode.poofyutils.geometry.EulerAngles;

interface IMU {

    void update();

    double getRoll();

    double getPitch();

    double getYaw();

    EulerAngles getAngles();

}
