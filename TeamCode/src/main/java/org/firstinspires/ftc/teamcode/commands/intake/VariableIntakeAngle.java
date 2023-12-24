package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class VariableIntakeAngle extends CommandBase {

    public IntakeSubsystem m_intakeSubsystem;

    public InterpLUT touch;

    public DoubleSupplier touchpad;
    public double intakeUp;
    public double intakeDown;

    public VariableIntakeAngle(IntakeSubsystem intakeSubsystem, DoubleSupplier touchpad, double intakeUp, double intakeDown) {
        this.m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
        this.touchpad = touchpad;
        this.intakeUp = intakeUp;
        this.intakeDown = intakeDown;

        touch = new InterpLUT();
        touch.add(-1.001, intakeUp);
        touch.add(1.001, intakeDown);
        touch.createLUT();
    }

    @Override
    public void initialize() {
        super.initialize();
        m_intakeSubsystem.setPosition(touch.get(touchpad.getAsDouble()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
