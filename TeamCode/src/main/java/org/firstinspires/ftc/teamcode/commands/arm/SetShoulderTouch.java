package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class SetShoulderTouch extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    private final DoubleSupplier position;

    public SetShoulderTouch(ArmSubsystem armSubsystem, DoubleSupplier position) {
        m_armSubsystem = armSubsystem;
        //addRequirements(m_armSubsystem);
        this.position = position;
    }

    @Override
    public void execute() {
        m_armSubsystem.setShoulderPositionTouch(position.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
