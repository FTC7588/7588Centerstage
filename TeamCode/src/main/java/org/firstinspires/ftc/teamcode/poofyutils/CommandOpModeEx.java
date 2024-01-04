package org.firstinspires.ftc.teamcode.poofyutils;

import com.arcrobotics.ftclib.command.CommandOpMode;

public abstract class CommandOpModeEx extends CommandOpMode {

    boolean runOnce = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (!isStopRequested() && !isStarted()) {
            initLoop();
        }

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            if (!runOnce) {
                runOnce();
                runOnce = true;
            }
            run();
        }
        reset();
    }

    public abstract void initLoop();

    public abstract void runOnce();
}
