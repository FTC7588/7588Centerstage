package org.firstinspires.ftc.teamcode.commands.arm;

import static org.firstinspires.ftc.teamcode.Constants.ARM_AUTO;
import static org.firstinspires.ftc.teamcode.Constants.ARM_PIVOT_NORM_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.ARM_WRIST_TEST;
import static org.firstinspires.ftc.teamcode.Constants.FLOOR_WRIST;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class AutoArmBack extends CommandBase {

    private ArmSubsystem armSS;

    private ElapsedTime timer1;
    private boolean done = false;

    public AutoArmBack(ArmSubsystem armSubsystem) {
        this.armSS = armSubsystem;
        addRequirements(armSubsystem);
        timer1 = new ElapsedTime();
    }

    @Override
    public void initialize() {
//        new SequentialCommandGroup(
//            new SetArmPositions(
//                    armSS,
//                    ARM_AUTO,
//                    0,
//                    ARM_PIVOT_NORM_DOWN
//            ),
//                new SetWristPosition(armSS, ARM_WRIST_TEST),
//                new WaitCommand(100),
//                new SetWristPosition(armSS, FLOOR_WRIST),
//                new WaitCommand(100),
//                new InstantCommand(() -> armSS.pivotPositionState = ArmSubsystem.PivotPositionState.MID)).schedule();

        armSS.setShoulderPosition(ARM_AUTO);
        armSS.setWristPosition(ARM_WRIST_TEST);
        armSS.pivotPositionState = ArmSubsystem.PivotPositionState.MID;
        timer1.reset();
    }

    @Override
    public void execute() {
        if (timer1.milliseconds() > 100) {
            armSS.setWristPosition(FLOOR_WRIST);
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
