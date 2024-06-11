package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ToggleArmStates extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ToggleArmStates(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        if (armSubsystem.armState == ArmSubsystem.ArmState.IDLE || armSubsystem.armState == ArmSubsystem.ArmState.GRAB) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> armSubsystem.setShoulderState(ArmSubsystem.ShoulderState.DEPOSIT)),
                        new WaitCommand(200),
                        new InstantCommand(() -> armSubsystem.setArmState(ArmSubsystem.ArmState.DEPOSIT)),
                        new WaitCommand(200),
                        new InstantCommand(() -> armSubsystem.setPivotStates(ArmSubsystem.PivotRotatedState.NORMAL, ArmSubsystem.PivotPositionState.MID))
                    )
            );
        } else {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> armSubsystem.setArmState(ArmSubsystem.ArmState.IDLE)),
                            new WaitCommand(200),
                            new InstantCommand(() -> armSubsystem.setPivotStates(ArmSubsystem.PivotRotatedState.NORMAL, ArmSubsystem.PivotPositionState.UP))
                    )
            );
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
