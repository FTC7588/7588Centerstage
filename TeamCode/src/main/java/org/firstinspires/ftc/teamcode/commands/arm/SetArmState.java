package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetArmState extends CommandBase {

    private ArmSubsystem armSubsystem;
    private ArmSubsystem.ArmState state;

    public SetArmState(ArmSubsystem armSubsystem, ArmSubsystem.ArmState state) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
        this.state = state;
    }

    @Override
    public void initialize() {
        armSubsystem.setArmState(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
