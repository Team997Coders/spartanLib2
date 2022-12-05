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

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/**
 * Class for the command-based structure which allows for method execution at beginning and end of
 * the command, with a boolean lambda of when the command should end.
 */
public class ConditionalEndRunnableCommand extends CommandBase {
    private final Supplier<Boolean> endConditional;

    private final Runnable init;
    private final Runnable end;

    private boolean runsWhileDisabled = false;

    /**
     * Constructs a ConditionalEndRunnableCommand.
     *
     * @param init Method (with no parameters) to be called when this command is initialized. Will
     *     not be called if {@code null}.
     * @param end Method (with no parameters) to be called when this command ends. Will not be
     *     called if {@code null}.
     * @param endConditional Lambda of a boolean dictating whether this command should end.
     * @param toRequire Any subsystems required to be free for the methods.
     */
    public ConditionalEndRunnableCommand(
            Runnable init, Runnable end, Supplier<Boolean> endConditional, Subsystem... toRequire) {
        this.endConditional = endConditional;

        this.init = init;
        this.end = end;

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
        if (init != null) init.run();
    }

    @Override
    public void end(boolean interrupted) {
        if (end != null) end.run();
    }

    @Override
    public boolean isFinished() {
        return endConditional.get();
    }
}
