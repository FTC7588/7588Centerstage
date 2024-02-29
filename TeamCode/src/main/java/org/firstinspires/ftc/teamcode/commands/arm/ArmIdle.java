package org.firstinspires.ftc.teamcode.commands.arm;

import static org.firstinspires.ftc.teamcode.Constants.ARM_PIVOT_MID;
import static org.firstinspires.ftc.teamcode.Constants.ARM_SHOULDER_IDLE;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_SHOULDER;
import static org.firstinspires.ftc.teamcode.Constants.GRAB_WRIST;
import static org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem.PivotPositionState.UP;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmIdle extends CommandBase {

    private ArmSubsystem armSS;

    public ArmIdle(ArmSubsystem armSubsystem) {
        this.armSS = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSS.setShoulderPosition(GRAB_SHOULDER);
        armSS.setWristPosition(GRAB_WRIST);
        armSS.pivotPositionState = UP;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
