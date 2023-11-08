package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class ResetIMU extends CommandBase {

    private final RobotHardware robot;

    public ResetIMU(RobotHardware robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.resetIMU();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
