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

import static org.junit.Assert.assertEquals;

import edu.wpi.first.math.VecBuilder;
import java.util.List;
import org.junit.Test;

/** Tests for the various small math functions in the UtilityMath class. */
public class UtilityMathTests {
    private static final double epsilon = 0.0001;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // normalizeAngleRadians
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    @Test
    public void utilityMathNormalizeAngleRadiansReturnsZeroCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleRadians(0), epsilon);
    }

    @Test
    public void utilityMathNormalizeAngleRadiansReturnsPositiveCoterminalsCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleRadians(2 * Math.PI), epsilon);
        assertEquals(0.5 * Math.PI, UtilityMath.normalizeAngleRadians(2.5 * Math.PI), epsilon);
    }

    @Test
    public void utilityMathNormalizeAngleRadiansReturnsNegativeCoterminalsCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleRadians(-2 * Math.PI), epsilon);
        assertEquals(0.5 * Math.PI, UtilityMath.normalizeAngleRadians(-1.5 * Math.PI), epsilon);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // normalizeAngleDegrees
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    @Test
    public void utilityMathNormalizeAngleDegreesReturnsZeroCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleDegrees(0), epsilon);
    }

    @Test
    public void utilityMathNormalizeAngleDegreesReturnsPositiveCoterminalsCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleDegrees(360), epsilon);
        assertEquals(90, UtilityMath.normalizeAngleDegrees(450), epsilon);
    }

    @Test
    public void utilityMathNormalizeAngleDegreesReturnsNegativeCoterminalsCorrectly() {
        assertEquals(0, UtilityMath.normalizeAngleDegrees(-360), epsilon);
        assertEquals(90, UtilityMath.normalizeAngleDegrees(-270), epsilon);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // smallestAngleRadiansBetween
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @Test
    public void utilityMathSmallestAngleRadiansBetweenWorksProperly() {
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
    public void utilityMathSmallestAngleDegreesBetweenWorksProperly() {
        assertEquals(0, UtilityMath.smallestAngleDegreesBetween(360, 0), epsilon);
        assertEquals(-90, UtilityMath.smallestAngleDegreesBetween(270, 180), epsilon);
        assertEquals(45, UtilityMath.smallestAngleDegreesBetween(315, 360), epsilon);
        assertEquals(-180, UtilityMath.smallestAngleDegreesBetween(180, 0), epsilon);
        assertEquals(-180, UtilityMath.smallestAngleDegreesBetween(-540, 0), epsilon);
        assertEquals(45, UtilityMath.smallestAngleDegreesBetween(-22.5, 22.5), epsilon);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // scaleToSum
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @Test
    public void utilityMathScaleToSetWorksProperly() {
        assertEquals(List.of(1.0), UtilityMath.scaleToSum(List.of(5.0), 1));

        assertEquals(
                List.of(5.0, 5.0, 5.0, 5.0, 5.0),
                UtilityMath.scaleToSum(List.of(17.0, 17.0, 17.0, 17.0, 17.0), 25));

        assertEquals(List.of(0.2, 0.8, 1.0), UtilityMath.scaleToSum(List.of(4.0, 16.0, 20.0), 2));

        assertEquals(List.of(-0.5, 5.0), UtilityMath.scaleToSum(List.of(1.0, -10.0), 4.5));
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // normalizeSet
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    @Test
    public void utilityMathNormalizeSetReturnsZeroCorrectly() throws Exception {
        List<Double> actualOutput = UtilityMath.normalizeSet(List.of(0.0), 1.0);

        assertEquals(List.of(0.0), actualOutput);
    }

    @Test
    public void utilityMathNormalizeSetAppliesCapProperly() throws Exception {
        List<Double> input = List.of(1.0);
        List<Double> expectedOutput = List.of(0.5);

        List<Double> actualOutput = UtilityMath.normalizeSet(input, 0.5);

        assertEquals(expectedOutput, actualOutput);
    }

    @Test
    public void utilityMathNormalizeSetWorksWithMultipleMembers() throws Exception {
        List<Double> input = List.of(0.5, 1.5, 3.0);
        List<Double> expectedOuput = List.of(0.25, 0.75, 1.5);
        List<Double> actualOutput = UtilityMath.normalizeSet(input, 1.5);

        assertEquals(expectedOuput, actualOutput);
    }

    @Test
    public void utilityMathNormalizeSetWorksWithNegatives() throws Exception {
        List<Double> input = List.of(-1.0, -2.0);
        List<Double> expectedOuput = List.of(-0.25, -0.5);
        List<Double> actualOutput = UtilityMath.normalizeSet(input, 0.5);

        assertEquals(expectedOuput, actualOutput);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // rotation matrices
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @Test
    public void utilityMath2DRotationMatrixWorks() {
        var inp = VecBuilder.fill(2, 1);

        var mat = UtilityMath.get2DRotationMatrix(0);

        assertEquals(inp, mat.times(inp));

        mat = UtilityMath.get2DRotationMatrix(Math.PI / 2);

        var res = mat.times(inp);

        assertEquals(-1, res.get(0, 0), epsilon);
        assertEquals(2, res.get(1, 0), epsilon);

        mat = UtilityMath.get2DRotationMatrix(-Math.PI / 4);

        res = mat.times(inp);

        assertEquals((3 * Math.sqrt(2) / 2), res.get(0, 0), epsilon);
        assertEquals(-Math.sqrt(2) / 2, res.get(1, 0), epsilon);
    }
}
