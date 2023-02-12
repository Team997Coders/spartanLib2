/**
Copyright 2022-2023 FRC Team 997

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
package org.chsrobotics.lib.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Class to simplify time-based iteration through a Sampleable object.
 *
 * @param T Type returned by the Sampleable object.
 */
public class IterativeSampler<T> {
    private final Sampleable<T> sampleable;

    private final Timer timer = new Timer();

    private final double offset;

    private final double timescale;

    /**
     * Constructs an IterativeSampler with a timescale of 1 second :: {@code timescale} units of
     * reference.
     *
     * <p>The internal timer starts paused, and must be explicitly started for iteration to work.
     *
     * @param sampleable Sampleable object to iterate through.
     * @param timescale Timescale. Must be > 0. Larger numbers make the iteration go faster, smaller
     *     numbers slower.
     */
    public IterativeSampler(Sampleable<T> sampleable, double timescale) {
        this.sampleable = sampleable;

        offset = sampleable.getMinReference();

        this.timescale = timescale;

        timer.stop();
    }

    /**
     * Constructs an IterativeSampler with a timescale of 1 second :: 1 unit of reference.
     *
     * <p>The internal timer starts paused, and must be explicitly started for iteration to work.
     *
     * @param sampleable Sampleable object to iterate through.
     */
    public IterativeSampler(Sampleable<T> sampleable) {
        this(sampleable, 1);
    }

    /** Starts the internal timer of the iterator. Safe to call if already started. */
    public void start() {
        timer.start();
    }

    /** Stops the internal timer of the iterator. Safe to call if already stopped. */
    public void pause() {
        timer.stop();
    }

    /**
     * Returns whether the iterator has finished iterating through the range of the sampleable.
     *
     * @return If the current reference is out of bounds of the sampleable.
     */
    public boolean isFinished() {
        return sampleable.isOutOfBounds(getReference());
    }

    /**
     * Returns the current value of the sampleable.
     *
     * @return The current value of the sampleable. If {@code isFinished()} is true, returns {@code
     *     null} instead.
     */
    public T getValue() {
        if (!isFinished()) return sampleable.sample(getReference());
        else return null;
    }

    /**
     * Returns the current reference used for sampling.
     *
     * @return The current reference. If {@code isFinished()} is true, returns {@code null} instead.
     */
    public double getReference() {
        if (!isFinished()) return (timescale * timer.get()) + offset;
        else return Double.NaN;
    }
}
