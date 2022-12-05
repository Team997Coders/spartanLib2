/**
Copyright 2022 FRC Team 997

This program is free software: 
you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with SpartanLib2. 
If not, see <https://www.gnu.org/licenses/>.
*/
package org.chsrobotics.lib.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Simple class to call a method (with no parameters) in the command-based structure. */
public class RunnableCommand extends CommandBase {
    private final Timer timer = new Timer();

    private final Runnable init;
    private final Runnable end;

    private final double durationSeconds;

    private boolean runsWhileDisabled = false;

    /**
     * Constructs a RunnableCommand.
     *
     * @param init Method to call upon initialization of this command. If {@code null}, will not be
     *     called.
     * @param end Method to call upon end of this command. If {@code null}, will not be called.
     * @param durationSeconds Time this command should be active for; if less than zero, will not
     *     end; if zero, will end in the first command scheduler loop.
     * @param toRequire Any subsystems that need to be free for the given methods.
     */
    public RunnableCommand(
            Runnable init, Runnable end, double durationSeconds, Subsystem... toRequire) {
        this.init = init;
        this.end = end;

        this.durationSeconds = durationSeconds;

        addRequirements(toRequire);
    }

    /**
     * Constructs a RunnableCommand that instantly ends.
     *
     * @param init The method to call upon initalization of this command. If {@code null}, will not
     *     be called.
     * @param toRequire Any subsystems that need to be free for the given method.
     */
    public RunnableCommand(Runnable init, Subsystem... toRequire) {
        this(init, null, 0);

        addRequirements(toRequire);
    }

    /**
     * Sets whether this command should continue to run even while the robot is in a disabled state.
     *
     * @param runsWhileDisabled True if this command should continue in the disabled state.
     */
    public void setRunsWhileDisabled(boolean runsWhileDisabled) {
        this.runsWhileDisabled = runsWhileDisabled;
    }

    @Override
    public boolean runsWhenDisabled() {
        return runsWhileDisabled;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if (init != null) init.run();
    }

    @Override
    public void end(boolean interrupted) {
        if (end != null) end.run();
    }

    @Override
    public boolean isFinished() {
        return (durationSeconds >= 0 && timer.get() >= durationSeconds);
    }
}
