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
package org.chsrobotics.lib.math;

import java.security.InvalidParameterException;

/** Various useful small math functions. */
public class UtilityMath {

    /**
     * Normalizes an angle in radians between 0 and 2 pi.
     *
     * @param angleRadians Value in radians of the angle.
     * @return The normalized value.
     */
    public static double normalizeAngleRadians(double angleRadians) {
        return ((angleRadians % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
        // remainder of the angle and 2pi, positive coterminal'd, and then remainder'd again
    }

    /**
     * Normalizes an angle in degrees between 0 and 360.
     *
     * @param angleDegrees Value of the angle.
     * @return The normalized value.
     */
    public static double normalizeAngleDegrees(double angleDegrees) {
        return ((angleDegrees % 360) + 360) % 360;
        // remainder of the angle and 360, positive coterminal'd, and then remainder'd again
    }

    /**
     * Interpolates a value between two points.
     *
     * <p>For more complex, multi-point interpolation, use the {@link MultiPointInterpolator} class.
     *
     * @param y0 The initial value of the dependent variable.
     * @param x0 The initial value of the independent variable.
     * @param y1 The final value of the dependent variable.
     * @param x1 The final value of the independent variable.
     * @param x The place at which to sample for the returned value.
     * @return The linearly interpolated value.
     */
    public static double simpleLinearInterpolation(
            double y0, double x0, double y1, double x1, double x) {
        return y0 + ((x - x0) * (y1 - y0) / (x1 - x0));
        // starting value, plus the distance of the sample from the start multiplied by
        // the slope
    }

    /**
     * Scales a set of doubles symmetrically to ensure that none of them exceed a maximum absolute
     * value, while still maintaining the same ratio.
     *
     * @param inputs An array of the input values.
     * @param maxAbsoluteValue The maximum absolute value allowed for an output.
     * @return An array of the scaled values, in the same order as they were input.
     * @throws InvalidParameterException If the array is empty.
     */
    public static double[] normalizeSet(double[] inputs, double maxAbsoluteValue)
            throws InvalidParameterException {
        if (inputs.length == 0) {
            throw new InvalidParameterException("Values must contain an element!");
        }
        int highestIndex = 0; // find the largest absolute value element in the list
        for (int i = 0; i < inputs.length; i++) {
            if (Math.abs(inputs[highestIndex]) < Math.abs(inputs[i])) {
                highestIndex = i;
            }
        }
        if (Math.abs(inputs[highestIndex]) <= maxAbsoluteValue) {
            return inputs; // if it's <= the max absolute value, just return the inputs
        } else {
            double[] outputs = new double[inputs.length];
            double scalingFactor = maxAbsoluteValue / Math.abs(inputs[highestIndex]);
            for (int i = 0;
                    i < inputs.length;
                    i++) { // get the scaling factor and apply it to each element
                outputs[i] = inputs[i] * scalingFactor;
            }
            return outputs;
        }
    }
}
