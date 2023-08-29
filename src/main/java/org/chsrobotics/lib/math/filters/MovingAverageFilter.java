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

import java.util.ArrayList;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.util.SizedStack;

/** Filter which computes the arithmetic mean of a stream of data. */
public class MovingAverageFilter extends Filter {

    /** Enum of different definitions of the mean. */
    public enum MEAN_IMPLEMENTATION {
        /** Sum of terms / number of terms. */
        ARITHMETIC,

        /** The (number of terms)-th root of the product of terms. */
        GEOMETRIC,

        /** Reciprocal of arithmetic mean of reciprocals of terms. */
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
     * @param impl Implementation of the mean to use.
     */
    public MovingAverageFilter(int window, MEAN_IMPLEMENTATION impl) {
        stack = new SizedStack<>(window);
        this.impl = impl;
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value, double dtSeconds) {
        stack.push(value);

        if (impl == MEAN_IMPLEMENTATION.GEOMETRIC) {
            currentOutput = UtilityMath.geometricMean(new ArrayList<>(stack));
        } else if (impl == MEAN_IMPLEMENTATION.HARMONIC) {
            currentOutput = UtilityMath.harmonicMean(new ArrayList<>(stack));
        } else {
            currentOutput = UtilityMath.arithmeticMean(new ArrayList<>(stack));
        }

        return currentOutput;
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
