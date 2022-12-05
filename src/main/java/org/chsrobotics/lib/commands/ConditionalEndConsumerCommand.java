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
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.chsrobotics.lib.util.Tuple2;

/**
 * Class for the command-based structure which allows for method execution (using one or two values
 * of generic T) at beginning and end of the command, with a boolean lambda of when the command
 * should end.
 *
 * @param T The data type of the method parameter(s).
 */
public class ConditionalEndConsumerCommand<T> extends CommandBase {
    private final boolean isBiConsumer;

    private final Consumer<T> consumer;
    private final BiConsumer<T, T> biConsumer;

    private final Supplier<Boolean> endConditional;

    private final T init;
    private final T end;

    private final Tuple2<T> biInit;
    private final Tuple2<T> biEnd;

    private boolean runsWhileDisabled = false;

    /**
     * Constructs a ConditionalConsumerCommand for a method with one parameter.
     *
     * @param consumer A method which takes one parameter of type T.
     * @param init A value of T to feed to the consumer upon command initalization.
     * @param end A value of T to feed to the consumer upon command end.
     * @param endConditional A lambda of whether the command should be ended.
     * @param toRequire Any subsystems required to be freed for method execution.
     */
    public ConditionalEndConsumerCommand(
            Consumer<T> consumer,
            T init,
            T end,
            Supplier<Boolean> endConditional,
            Subsystem... toRequire) {
        isBiConsumer = false;

        this.consumer = consumer;
        this.biConsumer = null;

        this.init = init;
        this.end = end;

        this.biInit = null;
        this.biEnd = null;

        this.endConditional = endConditional;

        addRequirements(toRequire);
    }

    /**
     * Constructs a ConditionalConsumerCommand for a method with two parameters.
     *
     * @param consumer A method which takes two parameters, each of type T.
     * @param init A Tuple2 of Ts to feed to the consumer upon command initalization, in the order
     *     that they are in the actual method.
     * @param end A Tuple2 of Ts to feed to the consumer upon command end, in the order that they
     *     are in the actual method.
     * @param endConditional A lambda of whether the command should be ended.
     * @param toRequire Any subsystems required to be freed for method execution.
     */
    public ConditionalEndConsumerCommand(
            BiConsumer<T, T> biConsumer,
            Tuple2<T> init,
            Tuple2<T> end,
            Supplier<Boolean> endConditional,
            Subsystem... toRequire) {
        isBiConsumer = true;

        this.biConsumer = biConsumer;
        this.consumer = null;

        this.init = null;
        this.end = null;

        this.biInit = init;
        this.biEnd = end;

        this.endConditional = endConditional;

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
        if (isBiConsumer && biConsumer != null) {
            biConsumer.accept(biInit.firstValue(), biInit.secondValue());
        } else if (consumer != null) {
            consumer.accept(init);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (isBiConsumer && biEnd != null && biConsumer != null) {
            biConsumer.accept(biEnd.firstValue(), biEnd.secondValue());
        } else if (end != null && consumer != null) {
            consumer.accept(end);
        }
    }

    @Override
    public boolean isFinished() {
        return (endConditional.get());
    }
}
