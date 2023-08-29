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

/**
 * An IIR variation upon the moving average. Instead of moving outside of the window, a past value's
 * influence over the filter's current value approaches but never reaches zero.
 */
public class ExponentialMovingAverage extends Filter {

    private final double responseConstant;

    private double lastOutput = 0;

    /**
     * Constructs an ExponentialMovingAverage.
     *
     * @param responseConstant Parameter dictating how quickly the filter should react to a new
     *     value. Must be in [0,1] (inclusive). A {@code responseConstant} of 1 produces a filter
     *     that instantly takes on the value of the input each cycle. A {@code responseConstant} of
     *     0 gives a filter that never changes from its initial value (0 in this implementation).
     */
    public ExponentialMovingAverage(double responseConstant) {
        this.responseConstant = responseConstant;
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value, double dtSeconds) {
        lastOutput = (value * responseConstant) + ((1 - responseConstant) * lastOutput);

        return lastOutput;
    }

    @Override
    /** {@inheritDoc} */
    public void reset() {
        lastOutput = 0;
    }

    @Override
    /** {@inheritDoc} */
    public double getCurrentOutput() {
        return lastOutput;
    }
}
