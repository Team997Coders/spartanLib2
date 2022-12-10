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

import org.chsrobotics.lib.util.Tuple2;

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
     * Returns the angle in radians between two other angles with the smallest absolute value.
     *
     * <p>A negative number indicates a clockwise rotation from angle a to angle b.
     *
     * @param angleA The first angle, in radians.
     * @param angleB The second angle, in raidans.
     * @return A signed angle in radians representing the smallest (positive is counterclockwise)
     *     angle between angles a and b.
     */
    public static double smallestAngleRadiansBetween(double angleA, double angleB) {
        double normA = UtilityMath.normalizeAngleRadians(angleA);
        double normB = UtilityMath.normalizeAngleRadians(angleB);

        double diff = (normB - normA + Math.PI) % (2 * Math.PI) - Math.PI;
        return (diff < -Math.PI) ? diff + (2 * Math.PI) : diff;
    }

    /**
     * Returns the angle in degrees between two other angles with the smallest absolute value.
     *
     * <p>A negative number indicates a clockwise rotation from angle a to angle b.
     *
     * @param angleA The first angle, in degrees.
     * @param angleB The second angle, in degrees.
     * @return A signed angle in degrees representing the smallest (positive is counterclockwise)
     *     angle between angles a and b.
     */
    public static double smallestAngleDegreesBetween(double angleA, double angleB) {
        double normA = UtilityMath.normalizeAngleDegrees(angleA);
        double normB = UtilityMath.normalizeAngleDegrees(angleB);

        double diff = (normB - normA + 180) % 360 - 180;
        return (diff < -180) ? diff + 360 : diff;
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
     * Scales a set of doubles symmetrically such that they sum to a desired number, while
     * maintaining the same ratio.
     *
     * @param inputs An array of doubles. If empty, this will return an empty array.
     * @param desiredSum The desired sum, positive or negative, of the outputs. If equal to zero,
     *     this will return an array of zeros of length equal to the input's length.
     * @return An array of doubles with the same ratios between each other as the inputs.
     */
    public static double[] scaleToSum(double[] inputs, double desiredSum) {
        if (inputs.length == 0) return inputs;
        if (desiredSum == 0) return new double[inputs.length];

        double sum = 0;
        for (double value : inputs) {
            sum += value;
        }

        double[] outputs = new double[inputs.length];

        double scalingFactor = desiredSum / sum;

        for (int i = 0; i < inputs.length; i++) {
            outputs[i] = inputs[i] * (scalingFactor);
        }

        return outputs;
    }

    /**
     * Scales a set of doubles symmetrically to ensure that none of them exceed a maximum absolute
     * value, while still maintaining the same ratio.
     *
     * @param inputs An array of the input values. If empty, this will return an empty array.
     * @param maxAbsoluteValue The maximum absolute value allowed for an output.
     * @return An array of the scaled values, in the same order as they were input.
     */
    public static double[] normalizeSet(double[] inputs, double maxAbsoluteValue) {
        if (inputs.length == 0) return inputs;
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

    /**
     * Constrains a value between two other numbers.
     *
     * @param ceiling The maximum allowed value of the output.
     * @param floor The minimum allowed value of the output.
     * @param value The value to constrain.
     * @return The constrained value.
     */
    public static double clamp(double ceiling, double floor, double value) {
        if (value > ceiling) return ceiling;
        else if (value < floor) return floor;
        else return value;
    }

    /**
     * Constrains a value under a maximum absolute value.
     *
     * @param maxValueAbs The maximum allowed absolute value of the output.
     * @param value The value to constrain.
     * @return The constrained value.
     */
    public static double clamp(double maxValueAbs, double value) {
        return clamp(Math.abs(maxValueAbs), -Math.abs(maxValueAbs), value);
    }

    /**
     * Finds the zeros (x-intercepts) of a quadratic (parabolic or lower-order polynomial) function
     * of the form y(x) = ax^2 + bx + c.
     *
     * <p>If there are no real-valued solutions, this will return {@code NaN} represented twice in
     * the Tuple.
     *
     * <p>If there is one real-valued solution, this will return that solution represented twice in
     * the Tuple.
     *
     * @param coeffA The coefficient of the ax^2 term of the quadratic polynomial.
     * @param coeffB The coefficient of the bx term of the quadratic polynomial.
     * @param coeffC The coefficient of the c (constant) term of the quadratic polynomial.
     * @return A Tuple2 holding up to 2 unique real solutions (if they exist).
     */
    public static Tuple2<Double> quadraticZeros(double coeffA, double coeffB, double coeffC) {
        double sqrtSafety = (coeffB * coeffB) - (4 * coeffA * coeffC);

        if (sqrtSafety < 0) return Tuple2.of(Double.NaN, Double.NaN);

        double pos = (-coeffB + Math.pow(sqrtSafety, 0.5)) / (2 * coeffA);
        double neg = (-coeffB - Math.pow(sqrtSafety, 0.5)) / (2 * coeffA);

        return Tuple2.of(pos, neg);
    }
}
