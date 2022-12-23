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
import java.util.function.Supplier;

/**
 * Class to allow execution of methods (with no parameters) upon boolean state change, in the
 * command-based structure.
 */
public class ConditionalRunnableCommand extends CommandBase {
    private final Timer timer = new Timer();

    private final Supplier<Boolean> conditional;

    private final Runnable onTrue;

    private final Runnable onFalse;

    private final double durationSeconds;

    private boolean lastConditionalValue;

    private boolean runsWhileDisabled = false;

    /**
     * Constructs a ConditionalRunnableCommand.
     *
     * @param onTrue Method to be called when {@code conditional} changes from {@code false} to
     *     {@code true}. If {@code null}, will not be called.
     * @param onFalse Method to be called when {@code conditional} changes from {@code true} to
     *     {@code false}. If {@code null}, will not be called.
     * @param conditional Lambda of a boolean state dictating when the runnables should be called.
     * @param durationSeconds Time that this command should be active for; if less than zero, this
     *     command will not end; if zero, this command will start and end in the same command
     *     scheduler loop.
     * @param toRequire Any subsystems required to be freed for the method.
     */
    public ConditionalRunnableCommand(
            Runnable onTrue,
            Runnable onFalse,
            Supplier<Boolean> conditional,
            double durationSeconds,
            Subsystem... toRequire) {
        this.conditional = conditional;

        this.onTrue = onTrue;
        this.onFalse = onFalse;

        this.durationSeconds = durationSeconds;

        lastConditionalValue = conditional.get();

        addRequirements(toRequire);
    }

    /**
     * Constructs a ConditionalRunnableCommand that does not end.
     *
     * @param onTrue Method to be called when {@code conditional} changes from {@code false} to
     *     {@code true}. If {@code null}, will not be called.
     * @param onFalse Method to be called when {@code conditional} changes from {@code true} to
     *     {@code false}. If {@code null}, will not be called.
     * @param conditional Lambda of a boolean state dictating when the runnables should be called.
     * @param toRequire Any subsystems required to be freed for the method.
     */
    public ConditionalRunnableCommand(
            Runnable onTrue,
            Runnable onFalse,
            Supplier<Boolean> conditional,
            Subsystem... toRequire) {
        this(onTrue, onFalse, conditional, -1, toRequire);
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
    }

    @Override
    public void execute() {
        boolean value = conditional.get();

        if (!lastConditionalValue && value && onTrue != null) {
            onTrue.run();
        } else if (lastConditionalValue && !value && onFalse != null) {
            onFalse.run();
        }
        lastConditionalValue = value;
    }

    @Override
    public boolean isFinished() {
        return (durationSeconds >= 0 && timer.get() >= durationSeconds);
    }
}
