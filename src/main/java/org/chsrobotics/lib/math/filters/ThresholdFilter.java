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

/**
 * A filter that completely attentuates any value (infinite impulse) below or above a defined
 * threshold.
 */
public class ThresholdFilter extends Filter {
    private final double threshold;
    private final boolean invert;
    private double currentValue;

    /**
     * Constructs a ThresholdFilter. By default, filters out values above the threshold, but can be
     * interved to filter out values below the threshold.
     *
     * @param threshold The filter will attentuate values above this threshold.
     * @param invert Whether to attentuate values *below* this threshold instead.
     */
    public ThresholdFilter(double threshold, boolean invert) {
        this.threshold = threshold;
        this.invert = invert;
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value) {
        if (!invert) {
            currentValue = (value <= threshold) ? value : 0;
        } else {
            currentValue = (value >= threshold) ? value : 0;
        }
        return currentValue;
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value, double dtSeconds) {
        return calculate(value);
    }

    @Override
    /** {@inheritDoc} */
    public void reset() {
        currentValue = 0;
    }

    @Override
    /** {@inheritDoc} */
    public double getCurrentOutput() {
        return currentValue;
    }
}
