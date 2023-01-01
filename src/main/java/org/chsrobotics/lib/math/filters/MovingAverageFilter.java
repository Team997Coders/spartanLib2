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
package org.chsrobotics.lib.math.filters;

import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.util.SizedStack;

/** Filter which computes the arithmetic mean of a stream of data. */
public class MovingAverageFilter extends Filter {

    /** Enum of different definitions of the mean. */
    public enum MEAN_IMPLEMENTATION {
        ARITHMETIC,
        GEOMETRIC,
        HARMONIC
    }

    private final SizedStack<Double> stack;

    private double currentOutput = 0;

    private final MEAN_IMPLEMENTATION impl;

    /**
     * Constructs a MovingAverageFilter.
     *
     * @param window Number of values to look back when calculating the average. If zero or
     *     negative, will be an indefinite window.
     */
    public MovingAverageFilter(int window, MEAN_IMPLEMENTATION impl) {
        stack = new SizedStack<>(window);
        this.impl = impl;
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value) {
        stack.push(value);

        double[] asArray = new double[stack.size()];

        for (int i = 0; i < stack.size(); i++) {
            asArray[i] = stack.get(i);
        }

        if (impl == MEAN_IMPLEMENTATION.GEOMETRIC) {
            currentOutput = UtilityMath.geometricMean(asArray);
        } else if (impl == MEAN_IMPLEMENTATION.HARMONIC) {
            currentOutput = UtilityMath.harmonicMean(asArray);
        } else {
            currentOutput = UtilityMath.arithmeticMean(asArray);
        }

        return currentOutput;
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value, double dtSeconds) {
        return calculate(value);
    }

    @Override
    /** {@inheritDoc} */
    public void reset() {
        stack.clear();
    }

    @Override
    /** {@inheritDoc} */
    public double getCurrentOutput() {
        return currentOutput;
    }
}
