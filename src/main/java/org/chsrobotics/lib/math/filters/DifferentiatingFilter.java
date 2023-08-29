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
package org.chsrobotics.lib.math.filters;

import org.chsrobotics.lib.math.UtilityMath;

/**
 * A filter that returns the rate of change (derivative) of a stream of data.
 *
 * <p>Approximated with finite timesteps.
 */
public class DifferentiatingFilter extends Filter {
    private double lastValue = 0;
    private double currentDeriv = 0;

    private final boolean angular;

    /**
     * Constructs a DifferentiatingFilter.
     *
     * @param angular Whether the given values are angles which can wrap. If true, the angles must
     *     be in radians. This will make the assumption that the smallest possible angle is
     *     traversed between steps, which may not necessarily be true for large dts.
     */
    public DifferentiatingFilter(boolean angular) {
        this.angular = angular;
    }

    /** Constructs a non-angular DifferentiatingFilter. */
    public DifferentiatingFilter() {
        this(false);
    }

    @Override
    /**
     * Calculates the rate of change (derivative) of a value relative to the filter's previous
     * input, using a given change in time.
     *
     * <p>The initial previous value is 0.
     *
     * <p>Returns 0 if {@code dt} is equal to 0 (although negative values of dt won't be meaningful
     * either).
     *
     * @param value The double value to input to the filter. If this filter is configured to be
     *     angular, must be in radians.
     * @param dt The elapsed time (in seconds) since the last call of either {@code calculate}
     *     method.
     * @return The rate of change, in units/second, between the current {@code value} and the
     *     previous {@code value}.
     */
    public double calculate(double value, double dtSeconds) {
        if (dtSeconds == 0) return 0;

        double delta;

        if (!angular) delta = value - lastValue;
        else delta = UtilityMath.smallestAngleRadiansBetween(lastValue, value);

        currentDeriv = delta / dtSeconds;
        lastValue = value;
        return currentDeriv;
    }

    /** {@inheritDoc} */
    @Override
    public void reset() {
        lastValue = 0;
        currentDeriv = 0;
    }

    /** {@inheritDoc} */
    @Override
    public double getCurrentOutput() {
        return currentDeriv;
    }
}
