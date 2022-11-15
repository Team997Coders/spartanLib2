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
 * A filter that attentuates any values (over time) below or above a defined threshold, while all
 * inputs passing said threshold are returned unchanged.
 *
 * <p>While the filter is in the process of attentuating its current value, if the value changes and
 * still cooresponds to attentuation, the attentuation will continue from where it was with the
 * previous value.
 *
 * <p>However, if the input passes the threshold and continues to do so after a change, the change
 * will be immediately reflected in the output.
 */
public class FIRThresholdFilter implements Filter {
    private final double response;
    private final double threshold;
    private final boolean invert;
    private double previousValue = 0;

    /**
     * Constructs a FIRThresholdFilter. By default, filters out values above the threshold, but can
     * be interved to filter out values below the threshold.
     *
     * @param response The maximum rate at which the output will go towards zero, per call of {@code
     *     calculate()}.
     * @param threshold The filter will attentuate values above this threshold.
     * @param invert Whether to attentuate values *below* this threshold instead.
     */
    public FIRThresholdFilter(double response, double threshold, boolean invert) {
        this.response = response;
        this.threshold = threshold;
        this.invert = invert;
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value) {
        // get the value *response* closer to 0
        double attentuated = ((Math.signum(value) * response) - previousValue);
        if (!invert) {
            if (value <= threshold) {
                // if passing the threshold, just return value
                previousValue = value;
            } else {
                // if the new value would overshoot, snap to 0
                if (Math.abs(previousValue) <= response) {
                    previousValue = 0;
                } else {
                    previousValue = attentuated;
                }
            }
        } else {
            if (value >= threshold) {
                previousValue = value;
            } else {
                if (Math.abs(previousValue) <= threshold) {
                    previousValue = 0;
                } else {
                    previousValue = attentuated;
                }
            }
        }
        return previousValue;
    }

    @Override
    /** {@inheritDoc} */
    public void reset() {
        previousValue = 0;
    }

    @Override
    /** {@inheritDoc} */
    public double getCurrentOutput() {
        return previousValue;
    }
}
