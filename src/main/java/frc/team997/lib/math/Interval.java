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
package frc.team997.lib.math;

/**
 * Representation of a simple interval of real numbers.
 *
 * <p>Unions of intervals can be made with boolean logic and multiple of these intervals.
 */
public class Interval {
    /** Whether a value equal to an endpoint should be considered part of the interval. */
    public enum Inclusion {
        INCLUSIVE,
        NONINCLUSIVE
    }

    private final Inclusion smallSideInclusion;
    private final Inclusion largeSideInclusion;
    private final double smallSide;
    private final double largeSide;

    /**
     * Creates an interval with the specified endpoints.
     *
     * <p>If the value of the leftSide is greater than the value of the rightSide, they will be
     * switched, retaining the same side/sideInclusion pairs.
     *
     * @param leftSideInclusion Whether the left side of the interval should include the endpoint.
     * @param leftSide The left endpoint.
     * @param rightSide The right endpoint.
     * @param rightSideInclusion Whether the right side of the interval should include the endpoint.
     */
    public Interval(
            Inclusion leftSideInclusion,
            Double leftSide,
            Double rightSide,
            Inclusion rightSideInclusion) {
        if (leftSide <= rightSide) {
            smallSideInclusion = leftSideInclusion;
            smallSide = leftSide;
            largeSideInclusion = rightSideInclusion;
            largeSide = rightSide;
        } else {
            smallSideInclusion = rightSideInclusion;
            smallSide = rightSide;
            largeSideInclusion = leftSideInclusion;
            largeSide = leftSide;
        }
    }

    /**
     * Returns whether the given number is a part of the interval.
     *
     * @param number The number to test.
     * @return Whether the provided number is included in the interval.
     */
    public boolean includes(double number) {
        if ((number == smallSide && smallSideInclusion == Inclusion.INCLUSIVE)
                || (number == largeSide && largeSideInclusion == Inclusion.INCLUSIVE)) {
            return true;
        } else return number > smallSide && number < largeSide;
    }
}
