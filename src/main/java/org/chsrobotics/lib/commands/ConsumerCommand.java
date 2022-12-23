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
import org.chsrobotics.lib.util.Tuple2;

/**
 * Simple class to allow calling methods with one or two parameters (of the same type) in the
 * command-based structure.
 *
 * @param T Data type of the method parameter or parameters.
 */
public class ConsumerCommand<T> extends CommandBase {
    private final boolean isBiConsumer;

    private final Consumer<T> consumer;
    private final BiConsumer<T, T> biConsumer;

    private final double durationSeconds;
    private final Timer timer = new Timer();

    private final T init;
    private final T end;

    private final Tuple2<T> biInit;
    private final Tuple2<T> biEnd;

    private boolean runsWhileDisabled = false;

    /**
     * Constructs a ConsumerCommand for a method with one parameter.
     *
     * @param consumer A method with a single parameter of type {@code T}.
     * @param init The value of {@code T} to feed to the consumer when this command is initialized.
     *     If {@code null}, will not be fed.
     * @param end The value of {@code T} to feed to the consumer when this command ends. If {@code
     *     null}, will not be fed.
     * @param durationSeconds Time that this command should be active; if less than zero, this
     *     command will not end; if zero, this command will end in the first command scheduler loop.
     * @param toRequire Any subsystems that need to be freed for the method.
     */
    public ConsumerCommand(
            Consumer<T> consumer, T init, T end, double durationSeconds, Subsystem... toRequire) {
        isBiConsumer = false;

        this.consumer = consumer;
        this.biConsumer = null;

        this.init = init;
        this.end = end;

        this.biInit = null;
        this.biEnd = null;

        this.durationSeconds = durationSeconds;

        addRequirements(toRequire);
    }

    /**
     * Constructs a ConsumerCommand for a method with a single parameter that instantly ends.
     *
     * @param consumer A method with a single parameter of type {@code T}.
     * @param init A value of type {@code T} to feed to the consumer when this command is
     *     initialized. If {@code null}, will not be fed.
     * @param toRequire Any subsystems that need to be freed for the method.
     */
    public ConsumerCommand(Consumer<T> consumer, T init, Subsystem... toRequire) {
        this(consumer, init, null, 0, toRequire);
    }

    /**
     * Constructs a ConsumerCommand for a method with two parameters.
     *
     * @param biConsumer A method with two parameters of type {@code T}.
     * @param init A Tuple2 of Ts to feed to the consumer when the command is initialized. The order
     *     should match the order of parameters in the actual method.
     * @param end A Tuple2 of Ts to feed to the consumer when the command ends. The order should
     *     match the order of parameters in the actual method.
     * @param durationSeconds How long this command should be active; if less than zero, this
     *     command won't end; if zero, this command will begin and end on the same command scheduler
     *     loop.
     * @param toRequire Any subsystems required to be freed for the method.
     */
    public ConsumerCommand(
            BiConsumer<T, T> biConsumer,
            Tuple2<T> init,
            Tuple2<T> end,
            double durationSeconds,
            Subsystem... toRequire) {

        isBiConsumer = true;

        this.biConsumer = biConsumer;
        this.consumer = null;

        this.init = null;
        this.end = null;

        this.biInit = init;
        this.biEnd = end;

        this.durationSeconds = durationSeconds;

        addRequirements(toRequire);
    }

    /**
     * Constructs a ConsumerCommand for a method with two parameters of the same type that ends
     * instantly.
     *
     * @param biConsumer A method with exactly two parameters of type {@code T}.
     * @param init A Tuple2 of Ts to feed to the consumer upon command initializaiton. The order
     *     should match the order of parameters in the actual method.
     * @param toRequire Any subsystems required to be freed for the method.
     */
    public ConsumerCommand(BiConsumer<T, T> biConsumer, Tuple2<T> init, Subsystem... toRequire) {
        this(biConsumer, init, null, 0, toRequire);
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
        return (durationSeconds >= 0 && timer.get() >= durationSeconds);
    }
}
