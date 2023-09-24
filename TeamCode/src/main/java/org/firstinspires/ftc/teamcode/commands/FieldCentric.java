package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class FieldCentric extends CommandBase {

    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier turnSpeed;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public FieldCentric(DrivetrainSubsystem drivetrainSubsystem,
                           DoubleSupplier strafeSpeed,
                           DoubleSupplier forwardSpeed,
                           DoubleSupplier turnSpeed)
    {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        this.strafeSpeed = strafeSpeed;
        this.forwardSpeed = forwardSpeed;
        this.turnSpeed = turnSpeed;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.fieldCentricMode(
                strafeSpeed.getAsDouble(),
                forwardSpeed.getAsDouble(),
                turnSpeed.getAsDouble(),
                false
        );
    }
}
