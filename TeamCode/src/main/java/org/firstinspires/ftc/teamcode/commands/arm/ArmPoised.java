package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmPoised extends CommandBase {

    private ArmSubsystem armSubsystem;

    public ArmPoised(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        armSubsystem.setShoulderPosition(Constants.POISED_SHOULDER);
        armSubsystem.setWristPosition(Constants.POISED_WRIST);
        armSubsystem.setPivotStates(ArmSubsystem.PivotRotatedState.NORMAL, ArmSubsystem.PivotPositionState.UP);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
