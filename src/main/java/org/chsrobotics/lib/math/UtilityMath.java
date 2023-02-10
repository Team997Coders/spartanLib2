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
package org.chsrobotics.lib.math;

import org.chsrobotics.lib.math.geometry.Vector3D;
import org.chsrobotics.lib.util.Tuple2;

/** Various useful small math functions. */
public class UtilityMath {
    public static final double defaultProportionEpsilon = 1E-3;
    public static final double defaultAbsoluteEpsilon = 1E-5;

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
     * Finds a midpoint between two other points in up to three-dimensional space.
     *
     * @param startpoint A vector with endpoint representing the starting point.
     * @param endpoint A vector with endpoint representing the ending point.
     * @param reference The place, in [0,1] (inclusive), to sample to find the midpoint.
     * @return A new vector with endpoint representing the interpolated midpoint.
     */
    public static Vector3D linearInterpolation(
            Vector3D startpoint, Vector3D endpoint, double reference) {
        return new Vector3D(
                startpoint.getX() + ((endpoint.getX() - startpoint.getX()) * reference),
                startpoint.getY() + ((endpoint.getY() - startpoint.getY()) * reference),
                startpoint.getZ() + ((endpoint.getZ() - startpoint.getZ()) * reference));
    }

    /**
     * Scales a set of doubles symmetrically such that they sum to a desired number, while
     * maintaining the same ratio.
     *
     * @param inputs An array of doubles. If empty, this will return an empty array. If this sums to
     *     zero, this will return this.
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

        if (sum == 0) return inputs;

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
     * Finds the zeros (x-intercepts) of a quadratic (second- or lower-order polynomial) function of
     * the form y(x) = ax^2 + bx + c.
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

    /**
     * Returns whether two values are equal to within an epsilon (calculated by their difference and
     * expected from floating-point arithmetic). Essentially just "close enough".
     *
     * @param valueA The first number to compare.
     * @param valueB The second number to compare.
     * @param absoluteEpsilon The maximum allowed difference between the two to return true.
     * @return Whether two values are "close enough" to each other.
     */
    public static boolean epsilonEqualsAbsolute(
            double valueA, double valueB, double absoluteEpsilon) {
        return (Math.abs(valueA - valueB) <= absoluteEpsilon);
    }

    /**
     * Returns whether two values are equal to within an epsilon (calculated by their difference and
     * expected from floating-point arithmetic). Essentially just "close enough".
     *
     * <p>Uses a default epsilon value of 1E-5 (0.00001).
     *
     * @param valueA The first number to compare.
     * @param valueB The second number to compare.
     * @return Whether two values are "close enough" to each other.
     */
    public static boolean epsilonEqualsAbsolute(double valueA, double valueB) {
        return epsilonEqualsAbsolute(valueA, valueB, defaultAbsoluteEpsilon);
    }

    /**
     * Returns whether two values are equal to within an epsilon (calculated multiplicatively and
     * expected from floating-point arithmetic). Essentially just "close enough".
     *
     * <p>To avoid division by zero, if only one of the two terms is exactly equal to zero, this
     * will return false. If *both* are exactly equal to zero, this returns true.
     *
     * @param valueA The first number to compare.
     * @param valueB The second number to compare.
     * @param proportionEpsilon If one value is bigger than another by a factor of at least {@code 1
     *     + proportionEpsilon}, this will return false.
     * @return Whether two values are "close enough" to each other.
     */
    public static boolean epsilonEqualsProportion(
            double valueA, double valueB, double proportionEpsilon) {
        if (valueA == 0 && valueB == 0) return true;
        if (valueA == 0 || valueB == 0) return false;
        return (Math.abs((valueA / valueB) - 1) <= proportionEpsilon);
    }

    /**
     * Returns whether two values are equal to within an epsilon (calculated multiplicatively and
     * expected from floating-point arithmetic). Essentially just "close enough".
     *
     * <p>Uses a default epsilon value of 1E-3 (0.001).
     *
     * <p>To avoid division by zero, if only one of the two terms is exactly equal to zero, this
     * will return false. If *both* are exactly equal to zero, this returns true.
     *
     * @param valueA The first number to compare.
     * @param valueB The second number to compare.
     * @return Whether two values are "close enough" to each other.
     */
    public static boolean epsilonEqualsProportion(double valueA, double valueB) {
        return epsilonEqualsProportion(valueA, valueB, defaultProportionEpsilon);
    }

    /**
     * Returns the solution of the Pythagorean formula for the hypotenuse of a right triangle, with
     * the leg lengths given.
     *
     * @param legLengthA The length of one of the legs.
     * @param legLengthB The length of the other leg.
     * @return The length of the hypotenuse.
     */
    public static double hypotenuse(double legLengthA, double legLengthB) {
        return Math.pow((legLengthA * legLengthA) + (legLengthB * legLengthB), 0.5);
    }

    /**
     * Returns the solution of the Pythagorean formula for a leg of a right triangle, with the other
     * leg length and the hypotenuse length given.
     *
     * @param hypotenuse The length of the hypotenuse.
     * @param legLengthA The length of one of the legs.
     * @return The length of the hypotenuse.
     */
    public static double leg(double hypotenuse, double legLengthA) {
        return Math.pow((hypotenuse * hypotenuse) - (legLengthA * legLengthA), 0.5);
    }

    /**
     * Calculates a higher order root of a number.
     *
     * @param value The value to take the root of.
     * @param order The order of the root (2 is a square root, 3 is a cubic root, etc).
     * @return The positive root. If {@code order} is equal to 0, will return {@code NaN}.
     */
    public static double root(double value, int order) {
        if (order == 0) return Double.NaN;
        return Math.pow(value, 1 / ((double) order));
    }

    /**
     * Takes the trigonometric cosecant of an angle.
     *
     * @param thetaRadians The angle. If coterminal to 0 or pi, this will return {@code NaN}.
     * @return The cosecant of the angle.
     */
    public static double csc(double thetaRadians) {
        if (Math.sin(thetaRadians) == 0) return Double.NaN;

        return 1 / Math.sin(thetaRadians);
    }

    /**
     * Takes the trigonometric secant of an angle.
     *
     * @param thetaRadians The angle. If coterminal to pi/2 or -pi/2, this will return {@code NaN}.
     * @return The secant of the angle.
     */
    public static double sec(double thetaRadians) {
        if (Math.cos(thetaRadians) == 0) return Double.NaN;

        return 1 / Math.cos(thetaRadians);
    }

    /**
     * Takes the trigonometric cotangent of an angle.
     *
     * @param thetaRadians The angle. If coterminal to 0 or pi, this will return {@code NaN}.
     * @return The cotangent of the angle.
     */
    public static double cot(double thetaRadians) {
        if (Math.sin(thetaRadians) == 0) return Double.NaN;

        return Math.cos(thetaRadians) / Math.sin(thetaRadians);
    }

    /**
     * Returns the trigonometric arccosecant (inverse cosecant) of a ratio of right triangle sides.
     *
     * @param ratio The ratio. If equal to 0, this will return {@code NaN}.
     * @return The arccosecant of the ratio.
     */
    public static double acsc(double ratio) {
        if (ratio == 0) return Double.NaN;

        return Math.asin(1 / ratio);
    }

    /**
     * Returns the trigonometric arcsecant (inverse secant) of a ratio of right triangle sides.
     *
     * @param ratio The ratio. If equal to 0, this will return {@code NaN}.
     * @return The arcsecant of the ratio.
     */
    public static double asec(double ratio) {
        if (ratio == 0) return Double.NaN;

        return Math.acos(1 / ratio);
    }

    /**
     * Returns the trigonometric arccotangent (inverse cotangent) of a ratio of right triangle
     * sides.
     *
     * @param ratio The ratio. If equal to 0, this will return {@code NaN}.
     * @return The arccotangent of the ratio.
     */
    public static double acot(double ratio) {
        if (ratio == 0) return Double.NaN;

        return Math.atan(1 / ratio);
    }

    /**
     * Two-argument arccotangent (inverse cotangent). This is useful over the single argument
     * arccotangent to avoid sign erasure (allowing returned angles in quadrange III instead of just
     * I).
     *
     * @param x A side length.
     * @param y A side length.
     * @return The angle associated with the sides.
     */
    public static double acot2(double x, double y) {
        return Math.atan2(y, x);
    }

    /**
     * Returns if a number is between (inclusive) the boundary numbers. The boundaries do not have
     * to be ordered.
     *
     * @param boundaryA The first boundary value.
     * @param boundaryB The second boundary value.
     * @param toCheck The value to check.
     * @return Whether the value is between (inclusive) the boundary numbers in either ordering.
     */
    public static boolean inRange(double boundaryA, double boundaryB, double toCheck) {
        return (((boundaryA <= toCheck) && (toCheck <= boundaryB))
                || ((boundaryA >= toCheck) && (toCheck >= boundaryB)));
    }

    /**
     * Converts a point in polar coordinates into a point in Cartesian coordinates.
     *
     * @param angle The angle, in radians counterclockwise from the positive x-axis, of the point
     *     vector.
     * @param magnitude The magnitude of the point vector.
     * @return A Tuple2 of the Cartesian coordinates, as (x, y).
     */
    public static Tuple2<Double> fromPolarToCartesian(double angle, double magnitude) {
        return Tuple2.of(magnitude * Math.cos(angle), magnitude * Math.sin(angle));
    }

    /**
     * Converts a point in Cartesian coordinates into a point in polar coordinates.
     *
     * @param x The x-value of the point.
     * @param y The y-value of the point.
     * @return A Tuple2 of the polar coordinates, as (angle, magnitude).
     */
    public static Tuple2<Double> fromCartesianToPolar(double x, double y) {
        return Tuple2.of(Math.atan2(y, x), hypotenuse(x, y));
    }

    /**
     * Computes the arithmetic mean, often called the average, of a set of numbers.
     *
     * @param values The numbers to find the arithmetic mean of. If empty, this will return {@code
     *     0}.
     * @return The arithmetic mean.
     */
    public static double arithmeticMean(double[] values) {
        if (values.length == 0) return 0;

        double sum = 0;

        for (double entry : values) {
            sum += entry;
        }

        return (sum / values.length);
    }

    /**
     * Computes the geometric mean of a series of numbers.
     *
     * @param values The numbers to find the geometric mean of. If empty, this will return {@code
     *     0}.
     * @return The geometric mean.
     */
    public static double geometricMean(double[] values) {
        if (values.length == 0) return 0;

        double product = 1;

        for (double entry : values) {
            product = product * entry;
        }

        return Math.pow(product, 1 / values.length);
    }

    /**
     * Computes the harmonic mean of a series of numbers.
     *
     * <p>If a number input to this is 0, its reciprocal will be defined as 0, and if the arithmetic
     * mean before reciprocation is 0, this will return 0.
     *
     * @param values The numbers to find the harmonic mean of. If empty, this will return {@code 0}.
     * @return The geometric mean.
     */
    public static double harmonicMean(double[] values) {
        double[] reciprocals = new double[values.length];

        for (int i = 0; i < values.length; i++) {
            if (values[i] == 0) reciprocals[i] = 0;
            else reciprocals[i] = (1 / values[i]);
        }

        double aMean = arithmeticMean(reciprocals);

        if (aMean == 0) return 0;
        else return (1 / aMean);
    }
}
