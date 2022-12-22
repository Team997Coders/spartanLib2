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

import java.util.ArrayList;

/** Represents a closed shape with a definite number of sides in 2-dimensional space. */
public class Polygon {
    private final Vector2D[] vertices;

    /**
     * Constructs a Polygon.
     *
     * @param vertices An ordered set of vectors with endpoints representing the vertices of the
     *     polygon. Lines will be connected between these vertices in accordance with the order of
     *     them in this constructor. The last edge is connected automatically.
     */
    public Polygon(Vector2D... vertices) {
        this.vertices = vertices;
    }

    /**
     * Returns whether a point is inside (not on the edge of) a polygon.
     *
     * @param point A vector with endpoint representing the point.
     * @return Whether the point lies within the bounds of the polygon.
     */
    public boolean pointLiesWithin(Vector2D point) {
        if (vertices.length == 0) return false;

        // create sides between vertices

        ArrayList<Line2D> sides = new ArrayList<>();

        for (int i = 0; i < vertices.length - 1; i++) {
            sides.add(new Line2D(vertices[i], vertices[i + 1]));
        }

        sides.add(new Line2D(vertices[vertices.length - 1], vertices[0]));

        for (Line2D side : sides) {
            if (side.pointOn(point)) return true;
        }

        // project a line to the right of the point with a length of 2.147 billion (should be good
        // enough for the vast majority of cases)
        Line2D projection =
                new Line2D(point, new Vector2D(point.getX() + Integer.MAX_VALUE, point.getY()));

        int intersectionCounter = 0;

        for (Line2D side : sides) {
            // have to exclude horizontal lines to avoid case where a line that is just being
            // touched, not crossed, is counted
            if (side.intersects(projection)
                    && side.getDirectionRadians() != Math.PI
                    && side.getDirectionRadians() != 0) {
                intersectionCounter++;
            }
        }

        // if a point is inside the polygon, a projected line will intersect edges an odd number of
        // times
        return (intersectionCounter % 2 == 1);
    }
}
