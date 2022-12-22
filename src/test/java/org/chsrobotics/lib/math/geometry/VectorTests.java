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
package org.chsrobotics.lib.math.geometry;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class VectorTests {
    private final double epsilon = 0.0001;

    @Test
    public void VectorScalarMultiplicationWorks() {
        Vector3D vector = new Vector3D(4, -2, 2);

        assertEquals(new Vector3D(-8, 4, -4), vector.scalarMultiply(-2));
    }

    @Test
    public void VectorMagnitudeWorks() {
        Vector3D vector = new Vector3D(5, -2, 1);

        assertEquals(Math.sqrt(30), vector.getMagnitude(), epsilon);
    }

    @Test
    public void VectorAdditionWorks() {
        Vector3D vectorA = new Vector3D(1, -6, 20);
        Vector3D vectorB = new Vector3D(-1, -2, 3);

        assertEquals(new Vector3D(0, -8, 23), vectorA.add(vectorB));
    }

    @Test
    public void VectorDotProductWorks() {
        Vector3D vectorA = new Vector3D(1, 0, 4);
        Vector3D vectorB = new Vector3D(-3, 2, 6);

        assertEquals(21, vectorA.dotProduct(vectorB), epsilon);
    }

    @Test
    public void VectorCrossProductWorks() {
        Vector3D vectorA = new Vector3D(3, 2, -1);
        Vector3D vectorB = new Vector3D(2, 0, -4);

        assertEquals(new Vector3D(-8, 10, -4), vectorA.crossProduct(vectorB));
    }
}
