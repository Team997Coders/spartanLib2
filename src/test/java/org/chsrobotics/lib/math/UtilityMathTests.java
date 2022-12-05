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

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** Tests for the various small math functions in the UtilityMath class. */
public class UtilityMathTests {
    private static final double epsilon = 0.0001;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // normalizeAngleRadians
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    @Test
    public void UtilityMathNormalizeAngleRadiansReturnsZeroCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleRadians(0), epsilon);
    }

    @Test
    public void UtilityMathNormalizeAngleRadiansReturnsPositiveCoterminalsCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleRadians(2 * Math.PI), epsilon);
        assertEquals(0.5 * Math.PI, UtilityMath.normalizeAngleRadians(2.5 * Math.PI), epsilon);
    }

    @Test
    public void UtilityMathNormalizeAngleRadiansReturnsNegativeCoterminalsCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleRadians(-2 * Math.PI), epsilon);
        assertEquals(0.5 * Math.PI, UtilityMath.normalizeAngleRadians(-1.5 * Math.PI), epsilon);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // normalizeAngleDegrees
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    @Test
    public void UtilityMathNormalizeAngleDegreesReturnsZeroCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleDegrees(0), epsilon);
    }

    @Test
    public void UtilityMathNormalizeAngleDegreesReturnsPositiveCoterminalsCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleDegrees(360), epsilon);
        assertEquals(90, UtilityMath.normalizeAngleDegrees(450), epsilon);
    }

    @Test
    public void UtilityMathNormalizeAngleDegreesReturnsNegativeCoterminalsCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleDegrees(-360), epsilon);
        assertEquals(90, UtilityMath.normalizeAngleDegrees(-270), epsilon);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // smallestAngleRadiansBetween
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @Test
    public void UtilityMathSmallestAngleRadiansBetweenWorksProperly() {
        assertEquals(0, UtilityMath.smallestAngleRadiansBetween(2 * Math.PI, 0), epsilon);
        assertEquals(
                -0.5 * Math.PI,
                UtilityMath.smallestAngleRadiansBetween(1.5 * Math.PI, Math.PI),
                epsilon);
        assertEquals(
                0.25 * Math.PI,
                UtilityMath.smallestAngleRadiansBetween(1.75 * Math.PI, 2 * Math.PI),
                epsilon);
        assertEquals(-Math.PI, UtilityMath.smallestAngleRadiansBetween(Math.PI, 0), epsilon);
        assertEquals(-Math.PI, UtilityMath.smallestAngleRadiansBetween(-3 * Math.PI, 0), epsilon);
        assertEquals(
                0.25 * Math.PI,
                UtilityMath.smallestAngleRadiansBetween(-0.125 * Math.PI, 0.125 * Math.PI),
                epsilon);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // smallestAngleDegreesBetween
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @Test
    public void UtilityMathSmallestAngleDegreesBetweenWorksProperly() {
        assertEquals(0, UtilityMath.smallestAngleDegreesBetween(360, 0), epsilon);
        assertEquals(-90, UtilityMath.smallestAngleDegreesBetween(270, 180), epsilon);
        assertEquals(45, UtilityMath.smallestAngleDegreesBetween(315, 360), epsilon);
        assertEquals(-180, UtilityMath.smallestAngleDegreesBetween(180, 0), epsilon);
        assertEquals(-180, UtilityMath.smallestAngleDegreesBetween(-540, 0), epsilon);
        assertEquals(45, UtilityMath.smallestAngleDegreesBetween(-22.5, 22.5), epsilon);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // simpleLinearInterpolation
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @Test
    public void UtilityMathSimpleLinearInterpolationReturnsInBoundsCorrectly() {
        assertEquals(1, UtilityMath.simpleLinearInterpolation(0, 0, 2, 2, 1), epsilon);
    }

    @Test
    public void UtilityMathSimpleLinearInterpolationReturnsOutOfBoundsCorrectly() {
        assertEquals(-5, UtilityMath.simpleLinearInterpolation(3, -1, -1, 1, 3), epsilon);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // scaleToSum
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @Test
    public void UtilityMathScaleToSetWorksProperly() {
        assertArrayEquals(new double[] {1}, UtilityMath.scaleToSum(new double[] {5}, 1), epsilon);

        assertArrayEquals(
                new double[] {5, 5, 5, 5, 5},
                UtilityMath.scaleToSum(new double[] {17, 17, 17, 17, 17}, 25),
                epsilon);

        assertArrayEquals(new double[2], UtilityMath.scaleToSum(new double[] {2, 4}, 0), epsilon);

        assertArrayEquals(new double[0], UtilityMath.scaleToSum(new double[0], 17), epsilon);

        assertArrayEquals(
                new double[] {0.2, 0.8, 1},
                UtilityMath.scaleToSum(new double[] {4, 16, 20}, 2),
                epsilon);

        assertArrayEquals(
                new double[] {-0.5, 5},
                UtilityMath.scaleToSum(new double[] {1, -10}, 4.5),
                epsilon);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // normalizeSet
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    @Test
    public void UtilityMathNormalizeSetReturnsZeroCorrectly() throws Exception {
        double[] zeroArray = {0.0};
        double[] actualOutput = UtilityMath.normalizeSet(zeroArray, 1.0);
        assertArrayEquals(zeroArray, actualOutput, epsilon);
    }

    @Test
    public void UtilityMathNormalizeSetAppliesCapProperly() throws Exception {
        double[] inputArray = {1.0};
        double[] expectedOutput = {0.5};
        double[] actualOutput = UtilityMath.normalizeSet(inputArray, 0.5);

        assertArrayEquals(expectedOutput, actualOutput, epsilon);
    }

    @Test
    public void UtilityMathNormalizeSetWorksWithMultipleMembers() throws Exception {
        double[] inputArray = {0.5, 1.5, 3.0};
        double[] expectedOuput = {0.25, 0.75, 1.5};
        double[] actualOutput = UtilityMath.normalizeSet(inputArray, 1.5);

        assertArrayEquals(expectedOuput, actualOutput, epsilon);
    }

    @Test
    public void UtilityMathNormalizeSetWorksWithNegatives() throws Exception {
        double[] inputArray = {-1.0, -2.0};
        double[] expectedOuput = {-0.25, -0.5};
        double[] actualOutput = UtilityMath.normalizeSet(inputArray, 0.5);

        assertArrayEquals(expectedOuput, actualOutput, epsilon);
    }
}
