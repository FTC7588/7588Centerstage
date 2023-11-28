package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.poofyutils.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class BackdropTagSlide extends CommandBase {

    public DoubleSupplier range;
    public Alliance alliance;

    public DrivetrainSubsystem m_drivetrainSubsystem;

    public BackdropTagSlide(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier range, Alliance alliance) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        this.range = range;
        this.alliance = alliance;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.followBackdropMode(range.getAsDouble(), alliance);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
