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
package org.chsrobotics.lib.math.geometry;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class Line2DTests {
    @Test
    public void Line2DPointOnWorks() {
        Line2D vertLine = new Line2D(new Vector2D(0, 5));

        assertTrue(vertLine.pointOn(new Vector2D(0, 3)));
        assertTrue(!vertLine.pointOn(new Vector2D(1, 1)));
        assertTrue(!vertLine.pointOn(new Vector2D(0, -5)));

        Line2D horLine = new Line2D(new Vector2D(1, 1), new Vector2D(-1, 1));

        assertTrue(horLine.pointOn(new Vector2D(0, 1)));
        assertTrue(horLine.pointOn(new Vector2D(1, 1)));
        assertTrue(!horLine.pointOn(new Vector2D(-2, 1)));

        Line2D slopedLine = new Line2D(new Vector2D(-5, -5), new Vector2D(-8, -8));

        assertTrue(slopedLine.pointOn(new Vector2D(-5, -5)));
        assertTrue(slopedLine.pointOn(new Vector2D(-6.5, -6.5)));
        assertTrue(!slopedLine.pointOn(new Vector2D(0, 0)));
        assertTrue(!slopedLine.pointOn(new Vector2D(2, -5)));
        assertTrue(!slopedLine.pointOn(new Vector2D(-5, 2)));
    }

    @Test
    public void Line2DIntersectingWorks() {
        assertTrue(
                new Line2D(new Vector2D(0, 1))
                        .intersects(new Line2D(new Vector2D(0, 0.5), new Vector2D(0, 5))));

        assertTrue(
                !new Line2D(new Vector2D(0, 1))
                        .intersects(new Line2D(new Vector2D(1, 0), new Vector2D(1, 1))));

        assertTrue(
                new Line2D(new Vector2D(1, 0))
                        .intersects(new Line2D(new Vector2D(0.5, 0), new Vector2D(1.5, 0))));

        assertTrue(
                new Line2D(new Vector2D(0.5, 0), new Vector2D(1.5, 0))
                        .intersects(new Line2D(new Vector2D(1, 0))));

        assertTrue(
                !new Line2D(new Vector2D(1, 1), new Vector2D(1, 3))
                        .intersects(new Line2D(new Vector2D(1, 4), new Vector2D(1, 5))));

        assertTrue(
                new Line2D(new Vector2D(1, 0), new Vector2D(1, 5))
                        .intersects(new Line2D(new Vector2D(2, 2))));

        assertTrue(
                !new Line2D(new Vector2D(-1, 0))
                        .intersects(new Line2D(new Vector2D(1, 0), new Vector2D(1, 5))));

        assertTrue(
                new Line2D(new Vector2D(1, 1), new Vector2D(-4, 3))
                        .intersects(new Line2D(new Vector2D(0, 1), new Vector2D(-3, 8))));

        assertTrue(
                !new Line2D(new Vector2D(4, -3))
                        .intersects(new Line2D(new Vector2D(5, 6), new Vector2D(-4, 4))));
    }
}
