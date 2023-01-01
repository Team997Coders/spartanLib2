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
package org.chsrobotics.lib.trajectory.planning;

import org.chsrobotics.lib.math.geometry.Line2D;
import org.chsrobotics.lib.math.geometry.Polygon;
import org.chsrobotics.lib.math.geometry.Vector2D;

/**
 * Class to abstract a configuration space, or a graphical representation of all possible states of
 * a robot mechanism.
 */
public class ConfigurationSpace {

    /** A dimension of a configuration space. */
    public class ConfigurationSpaceDimension {
        /**
         * Minimum numerical value attainable in this dimension. If {@code wrap} is enabled, is
         * geometrically equivalent to {@code max}.
         */
        public final double min;

        /**
         * Maximum numerical value attainable in this dimension. If {@code wrap} is enabled, is
         * geometrically equivalent to {@code min}.
         */
        public final double max;

        /**
         * Whether values greater than {@code min} or {@code max} should be wrapped around to the
         * other side of the dimension. Most likely useful for angular dimensions.
         */
        public final boolean wrap;

        /**
         * Constructs a ConfigurationSpaceDimension.
         *
         * @param min
         * @param max
         * @param wrap
         */
        public ConfigurationSpaceDimension(double min, double max, boolean wrap) {
            this.min = min;
            this.max = max;
            this.wrap = wrap;
        }
    }

    private final ConfigurationSpaceDimension dimensionA;
    private final ConfigurationSpaceDimension dimensionB;

    private final Polygon[] obstacles;

    /**
     * Constructs a 2-dimensional ConfigurationSpace.
     *
     * @param dimensionA The first dimension of the configuration space. Given vector coordinates in
     *     the x-dimension will be in this dimension instead.
     * @param dimensionB The second dimension of the configuration space. Given vector coordinates
     *     in the y-dimension will be in this dimension instead.
     * @param obstacles Polygons within the space to be considered obstacles to a point or path
     *     through them.
     */
    public ConfigurationSpace(
            ConfigurationSpaceDimension dimensionA,
            ConfigurationSpaceDimension dimensionB,
            Polygon... obstacles) {
        this.dimensionA = dimensionA;
        this.dimensionB = dimensionB;

        this.obstacles = obstacles;
    }

    // unimplemented, but coming in the future

    // public ConfigurationSpace(String jsonString) {
    //     this(null, null);
    // }

    // public String generateJson() {
    //     return null;
    // }

    /**
     * Returns whether the given point is a valid point within the configuration space.
     *
     * @param point A vector with endpoint representing the point, such that its values coorespond
     *     to dimensionA and dimensionB, respectively.
     * @return Whether the point is outside of obstacles and inside the space (if wrap in that
     *     dimension is not enabled).
     */
    public boolean isValidPoint(Vector2D point) {
        boolean retVal = true;

        boolean inBoundsDimA =
                dimensionA.wrap
                        || (point.getX() <= dimensionA.max && point.getX() >= dimensionA.min);

        boolean inBoundsDimB =
                dimensionB.wrap
                        || (point.getY() <= dimensionB.max && point.getY() >= dimensionB.min);

        if (inBoundsDimA && inBoundsDimB) {
            for (Polygon poly : obstacles) {
                if (retVal) retVal = !poly.pointLiesWithin(point);
            }
        } else retVal = false;

        return retVal;
    }

    /**
     * Returns whether the given line intersects any obstacles in the configuration space.
     *
     * <p>Currently does not support dimension wrapping, and will therefore return {@code false} if
     * a line point is out of bounds, even if according to wrap it should be valid.
     *
     * @param line A line defined such that the point vectors are in the space (dimensionA,
     *     dimensionB), respectively.
     * @return Whether the given line intersects any obstacles.
     */
    public boolean intersectsObstacle(Line2D line) {
        boolean retVal = false;

        for (Polygon poly : obstacles) {
            if (!retVal) retVal = poly.lineIntersectsAnyEdge(line);
        }
        return (retVal || !isValidPoint(line.getStartPoint()) || !isValidPoint(line.getEndPoint()));
    }

    /**
     * Returns a (pseudo-) random point outside of any obstacles in the configuration space.
     *
     * @return A pseudorandomly selected valid point within the configuration space. The vector
     *     represents the point such that its values coorespond to dimensionA and dimensionB,
     *     respectively.
     */
    public Vector2D randomValidPoint() {
        double dimA = (Math.abs(dimensionA.max - dimensionA.min) * Math.random()) + dimensionA.min;
        double dimB = (Math.abs(dimensionB.max - dimensionB.min) * Math.random()) + dimensionB.min;

        Vector2D vec = new Vector2D(dimA, dimB);

        if (isValidPoint(vec)) return vec;
        else return randomValidPoint();
        // recurse until point is valid... will obviously cause stackoverflow if there are no valid
        // points but that's the user's fault
    }
}
