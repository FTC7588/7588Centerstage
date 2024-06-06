package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ToggleArmStates extends CommandBase {

    private ArmSubsystem armSubsystem;

    public ToggleArmStates(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.toggleArmState();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
