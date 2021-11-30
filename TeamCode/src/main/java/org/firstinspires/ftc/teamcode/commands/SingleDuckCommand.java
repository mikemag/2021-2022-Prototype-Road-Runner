package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DuckWheel;

public class SingleDuckCommand extends CommandBase {
    final DuckWheel duckWheel;

    boolean runningSlow = true;
    boolean done = false;

    public SingleDuckCommand(DuckWheel duckWheel) {
        this.duckWheel = duckWheel;
        addRequirements(duckWheel);
    }

    @Override
    public void initialize() {
        duckWheel.reset();
        duckWheel.runSlowArc();
        runningSlow = true;
        done = false;
    }

    @Override
    public void execute() {
        if (!duckWheel.isBusy()) {
            if (runningSlow) {
                duckWheel.runFastArc();
                runningSlow = false;
            } else {
                done = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean isInterrupted) {
        duckWheel.stop();
        done = false;
    }
}
