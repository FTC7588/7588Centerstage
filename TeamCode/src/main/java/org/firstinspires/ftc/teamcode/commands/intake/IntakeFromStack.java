package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeFromStack extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    private int topPixel;

    private double topPosition;

    private boolean jammed = false;

    private ElapsedTime failTime;
    private ElapsedTime jamTimer;

    public IntakeFromStack(IntakeSubsystem intakeSubsystem, int topPixel) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
        this.topPixel = topPixel;
        failTime = new ElapsedTime();
    }

    @Override
    public void initialize() {
        updateIntakeHeight();
        intakeSubsystem.setPower(-1);
        failTime.reset();
    }

    @Override
    public void execute() {
        if (failTime.seconds() > 1 && !jammed) {
            topPixel--;
            updateIntakeHeight();
            failTime.reset();
        } else if (intakeSubsystem.getAverageCurrent() > 4 && !jammed) {
            jammed = true;
            jamTimer.reset();
            intakeSubsystem.setPower(1);
        } else if (jammed && jamTimer.seconds() > 1) {
            intakeSubsystem.setPower(-1);
            jammed = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
//        intakeSubsystem.setPower(0);
        intakeSubsystem.setPosition(Constants.INT_UP);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isLoaded();
    }

    public void updateIntakeHeight() {
        switch (topPixel) {
            case 5:
                topPosition = Constants.INT_FIVE;
                break;
            case 4:
                topPosition = Constants.INT_FOUR;
                break;
            case 3:
                topPosition = Constants.INT_THREE;
                break;
            case 2:
                topPosition = Constants.INT_TWO;
                break;
            case 1:
                topPosition = Constants.INT_ONE;
                break;
            case 0:
                topPosition = Constants.INT_FIVE;
                topPixel = 5;
                break;
        }
        intakeSubsystem.setPosition(topPosition);
    }
}
