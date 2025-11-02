package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * A subsystem base wrapper which adds command factories, and a printTelemetry periodesque funcion
 */

public abstract class SubsystemBasePlus extends SubsystemBase {
    /**
     * Constructs a command that runs an action every iteration until interrupted, and then runs a second action. Requires this subsystem.
     * Parameters:
     * run - the action to run every iteration
     * end - the action to run on interrupt
     * Returns:
     * the command
     *
     * @return
     */
    public CommandBase runEnd(Runnable run, Runnable end) {
        CommandBase cb = new CommandBase() {
            @Override
            public void execute() {
                run.run();
            }

            @Override
            public void end(boolean onInterrupted) {
                end.run();
            }

        };
        cb.addRequirements(this);
        return cb;
    }

    /**
     * Constructs a command that runs an action once and another action when the command is interrupted. Requires this subsystem.
     * Parameters:
     * start - the action to run on start
     * end - the action to run on interrupt
     * Returns:
     * the command
     *
     * @param start
     * @param end
     * @return
     */
    public CommandBase startEnd(Runnable start, Runnable end) {
        CommandBase cb = new CommandBase() {
            @Override
            public void initialize() {
                start.run();
            }

            @Override
            public void end(boolean onInterrupted) {
                end.run();
            }

        };
        cb.addRequirements(this);
        return cb;
    }

    /**
     * default CommandBase runOnce​(Runnable action)
     * Constructs a command that runs an action once and finishes. Requires this subsystem.
     * Parameters:
     * action - the action to run
     * Returns:
     * the command
     *
     * @param action
     * @return
     */
    public CommandBase runOnce(Runnable action) {
        CommandBase cb = new CommandBase() {
            @Override
            public void execute() {
                action.run();
            }

            @Override
            public boolean isFinished() {
                return true;
            }

        };
        cb.addRequirements(this);
        return cb;
    }

    /**
     * method which when called prints all the telemetry data associated with this particular subsystem
     *
     * @param t
     */
    abstract void printTelemetry(Telemetry t);
}
