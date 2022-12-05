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
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.chsrobotics.lib.util.Tuple2;

/**
 * Class to allow calling methods with one or two parameters (of the same type), in the
 * command-based structure, upon state change of a boolean conditional.
 *
 * @param T Data type of the method parameter or parameters.
 */
public class ConditionalConsumerCommand<T> extends CommandBase {
    private final boolean isBiConsumer;

    private final Consumer<T> consumer;
    private final BiConsumer<T, T> biConsumer;

    private final Supplier<Boolean> conditional;

    private final double durationSeconds;
    private final Timer timer = new Timer();

    private final T onTrue;
    private final T onFalse;

    private final Tuple2<T> biOnTrue;
    private final Tuple2<T> biOnFalse;

    private boolean runsWhileDisabled = false;

    private boolean lastConditionalValue;

    /**
     * Constructs a ConditionalConsumerCommand for a method with one parameter.
     *
     * @param consumer A method which takes one parameter of type T.
     * @param onTrue A value of T to feed to the consumer upon conditional change from false to
     *     true.
     * @param onFalse A value of T to feed to the consumer upon conditional change from true to
     *     false.
     * @param conditional A lambda of a boolean.
     * @param durationSeconds Time that this command should expire after.
     * @param toRequire Any subsystems required to be freed for method execution.
     */
    public ConditionalConsumerCommand(
            Consumer<T> consumer,
            T onTrue,
            T onFalse,
            Supplier<Boolean> conditional,
            double durationSeconds,
            Subsystem... toRequire) {
        isBiConsumer = false;

        this.consumer = consumer;
        this.biConsumer = null;

        this.onTrue = onTrue;
        this.onFalse = onFalse;

        this.biOnTrue = null;
        this.biOnFalse = null;

        this.durationSeconds = durationSeconds;

        this.conditional = conditional;

        lastConditionalValue = conditional.get();

        addRequirements(toRequire);
    }

    /**
     * Constructs a ConditionalConsumerCommand for a method with one parameter which never ends.
     *
     * @param consumer A method which takes one parameter of type T.
     * @param onTrue A value of T to feed to the consumer upon conditional change from false to
     *     true.
     * @param onFalse A value of T to feed to the consumer upon conditional change from true to
     *     false.
     * @param conditional A lambda of a boolean.
     * @param toRequire Any subsystems required to be freed for method execution.
     */
    public ConditionalConsumerCommand(
            Consumer<T> consumer,
            T onTrue,
            T onFalse,
            Supplier<Boolean> conditional,
            Subsystem... toRequire) {
        this(consumer, onTrue, onFalse, conditional, -1, toRequire);
    }

    /**
     * Constructs a ConditionalConsumerCommand for a method with two parameter.
     *
     * @param biConsumer A method which takes two parameters of type T.
     * @param onTrue A Tuple2 of Ts to feed to the consumer upon conditional change from false to
     *     true. Should be in the same order as in the actual method.
     * @param onFalse A Tuple2 of Ts to feed to the consumer upon conditional change from true to
     *     false. Should be in the same order as in the actual method.
     * @param conditional A lambda of a boolean.
     * @param durationSeconds Time that this command should expire after.
     * @param toRequire Any subsystems required to be freed for method execution.
     */
    public ConditionalConsumerCommand(
            BiConsumer<T, T> biConsumer,
            Tuple2<T> onTrue,
            Tuple2<T> onFalse,
            double durationSeconds,
            Supplier<Boolean> conditional,
            Subsystem... toRequire) {
        isBiConsumer = true;

        this.biConsumer = biConsumer;
        this.consumer = null;

        this.onTrue = null;
        this.onFalse = null;

        this.biOnTrue = onTrue;
        this.biOnFalse = onFalse;

        this.durationSeconds = durationSeconds;

        this.conditional = conditional;

        addRequirements(toRequire);
    }

    /**
     * Constructs a ConditionalConsumerCommand for a method with two parameter which never ends.
     *
     * @param consumer A method which takes one parameters of type T.
     * @param onTrue A Tuple2 of T to feed to the consumer upon conditional change from false to
     *     true.
     * @param onFalse A Tuple2 of T to feed to the consumer upon conditional change from true to
     *     false.
     * @param conditional A lambda of a boolean.
     * @param toRequire Any subsystems required to be freed for method execution.
     */
    public ConditionalConsumerCommand(
            BiConsumer<T, T> consumer,
            Tuple2<T> onTrue,
            Tuple2<T> onFalse,
            Supplier<Boolean> conditional,
            Subsystem... toRequire) {
        this(consumer, onTrue, onFalse, -1, conditional, toRequire);
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
    public void execute() {
        boolean value = conditional.get();

        if (!lastConditionalValue && value && onTrue != null) {
            if (isBiConsumer) {
                biConsumer.accept(biOnTrue.firstValue(), biOnTrue.secondValue());
            } else {
                consumer.accept(onTrue);
            }
        } else if (lastConditionalValue && !value && onFalse != null) {
            if (isBiConsumer) {
                biConsumer.accept(biOnFalse.firstValue(), biOnFalse.secondValue());
            } else {
                consumer.accept(onFalse);
            }
        }
        lastConditionalValue = value;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return (durationSeconds >= 0 && timer.get() >= durationSeconds);
    }
}
