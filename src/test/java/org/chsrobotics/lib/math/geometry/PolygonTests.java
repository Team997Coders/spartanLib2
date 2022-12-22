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

import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PolygonTests {
    @Test
    public void PolygonWorksRectangle() {
        Polygon rectangle =
                new Polygon(
                        new Vector2D(0, 0),
                        new Vector2D(1, 0),
                        new Vector2D(1, 1),
                        new Vector2D(0, 1));

        assertTrue(rectangle.pointLiesWithin(new Vector2D(0.5, 0.5)));
        assertTrue(!rectangle.pointLiesWithin(new Vector2D(10, 0)));
        assertTrue(rectangle.pointLiesWithin(new Vector2D(0.5, 1)));
        assertTrue(rectangle.pointLiesWithin(new Vector2D(1, 1)));
    }

    @Test
    public void PolygonWorksTriangle() {
        Polygon triangle = new Polygon(new Vector2D(0, 0), new Vector2D(2, 0), new Vector2D(1, 1));

        assertTrue(triangle.pointLiesWithin(new Vector2D(0, 0)));
        assertTrue(triangle.pointLiesWithin(new Vector2D(1.5, 0)));
        assertTrue(triangle.pointLiesWithin(new Vector2D(1.5, 0.5)));
        assertTrue(!triangle.pointLiesWithin(new Vector2D(1.51, 0.5)));
    }
}
