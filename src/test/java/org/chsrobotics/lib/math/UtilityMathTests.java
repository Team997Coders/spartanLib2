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
import static org.junit.Assert.assertThrows;

import java.security.InvalidParameterException;
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
    // normalizeSet
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    @Test
    public void UtilityMathNormalizeSetHandlesEmptyListProperly() {
        double[] emptyArray = {};
        assertThrows(
                InvalidParameterException.class,
                () -> {
                    UtilityMath.normalizeSet(emptyArray, 0);
                });
    }

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
