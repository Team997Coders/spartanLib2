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

import org.chsrobotics.lib.math.UtilityMath;

/**
 * Class representing a line in 2-dimensional space, in the form of a vector. This differs from
 * Vector2D in that it does not necessarily have a startpoint at the origin, and that it contains
 * methods to calculate whether points lie on it or if other lines intersect it.
 */
public class Line2D {
    private final Vector2D startPoint;
    private final Vector2D endPoint;

    /**
     * Constructs a Line2D from polar form with a provided startpoint.
     *
     * @param origin A vector with endpoint representing the startpoint of the line.
     * @param magnitude The absolute length of the line.
     * @param directionRadians The angle of the line in radians (from start to endpoint).
     */
    public Line2D(Vector2D origin, double magnitude, double directionRadians) {
        this.startPoint = origin;

        this.endPoint =
                new Vector2D(
                        magnitude * Math.cos(directionRadians),
                        magnitude * Math.sin(directionRadians));
    }

    /**
     * Constructs a Line2D from polar form with a startpoint at the origin.
     *
     * @param magnitude The absolute length of the line.
     * @param directionRadians The angle of the line in radians (from origin to endpoint).
     */
    public Line2D(double magnitude, double directionRadians) {
        this(new Vector2D(0, 0), magnitude, directionRadians);
    }

    /**
     * Constructs a Line2D from Cartesian form with a provided startpoint.
     *
     * @param origin A vector with endpoint representing the startpoint of the line.
     * @param end A vector with endpoint representing the endpoint of the line.
     */
    public Line2D(Vector2D origin, Vector2D end) {
        this.startPoint = origin;
        this.endPoint = end;
    }

    /**
     * Constructs a Line2D from Cartesian form using the origin as the startpoint.
     *
     * @param end A vector with endpoint representing the endpoint of the line.
     */
    public Line2D(Vector2D end) {
        this(new Vector2D(0, 0), end);
    }

    /**
     * Returns the angle of the line in radians from the startpoint. If the line is vertical,
     * returns {@code NaN}.
     *
     * @return The angle of the vector from startpoint to endpoint.
     */
    public double getDirectionRadians() {
        if (getDeltaX() == 0) return Double.NaN;
        return Math.atan2(getDeltaY(), getDeltaX());
    }

    /**
     * Returns the absolute length of the line.
     *
     * @return The absolute distance between the startpoint and endpoint.
     */
    public double getMagnitude() {
        return endPoint.subtract(startPoint).getMagnitude();
    }

    /**
     * Returns the change in the X-axis from the start of the line to the end of the line.
     *
     * @return The difference in the X of the endpoint and the startpoint.
     */
    public double getDeltaX() {
        return endPoint.subtract(startPoint).getX();
    }

    /**
     * Returns the change in the Y-axis from the start of the line to the end of the line.
     *
     * @return The difference in the Y of the endpoint and the startpoint.
     */
    public double getDeltaY() {
        return endPoint.subtract(startPoint).getY();
    }

    /**
     * Returns whether a point lies upon the line.
     *
     * @param point A vector with endpoint representing the point.
     * @return Whether the point lies on the line within an epsilon.
     */
    public boolean pointOn(Vector2D point) {
        if (point.equals(startPoint) || point.equals(endPoint)) return true;

        // if this line is vertical, check if x matches and y within range
        if (getDeltaX() == 0)
            return (point.getX() == startPoint.getX()
                    && UtilityMath.inRange(startPoint.getY(), endPoint.getY(), point.getY()));

        // if this line is horizontal, check if y matches and x within range
        if (getDeltaY() == 0)
            return (point.getY() == startPoint.getY()
                    && UtilityMath.inRange(startPoint.getX(), endPoint.getX(), point.getX()));

        // if the line from the startpoint to the check point is vertical, the above cases would
        // already have caught it
        // needed to prevent div by 0 in the next check
        if (startPoint.getX() - point.getX() == 0) return false;

        // else, check that the slope of the line == that of the line from the start to the point
        boolean colinear =
                UtilityMath.epsilonEqualsAbsolute(
                        (getDeltaY()) / (getDeltaX()),
                        (point.getY() - startPoint.getY()) / (point.getX() - startPoint.getX()));

        // final check to ensure that the point is in the boundaries of the line
        return (colinear && UtilityMath.inRange(startPoint.getX(), endPoint.getX(), point.getX()))
                && UtilityMath.inRange(startPoint.getY(), endPoint.getY(), point.getY());
    }

    /**
     * Returns whether a line intersects this line.
     *
     * @param other The other line.
     * @return Whether the two lines intersect (colinearity over the same space counts).
     */
    public boolean intersects(Line2D other) {
        // check if lines are parallel
        if (UtilityMath.normalizeAngleRadians(this.getDirectionRadians())
                        == UtilityMath.normalizeAngleRadians(other.getDirectionRadians())
                || (Double.isNaN(this.getDirectionRadians())
                        && (Double.isNaN(other.getDirectionRadians())))) {

            // if parallel and intersecting, either this has its startpoint in the other, or the
            // other has its startpoint in this
            boolean otherStartpointInThis =
                    (UtilityMath.inRange(
                                    this.getStartPoint().getX(),
                                    this.getEndPoint().getX(),
                                    other.getStartPoint().getX())
                            && UtilityMath.inRange(
                                    this.getStartPoint().getY(),
                                    this.getEndPoint().getY(),
                                    other.getStartPoint().getY()));

            boolean thisStartpointInOther =
                    (UtilityMath.inRange(
                                    other.getStartPoint().getX(),
                                    other.getEndPoint().getX(),
                                    this.getStartPoint().getX())
                            && UtilityMath.inRange(
                                    other.getStartPoint().getY(),
                                    other.getEndPoint().getY(),
                                    this.getStartPoint().getY()));

            return (otherStartpointInThis || thisStartpointInOther);
        }

        // if one of the lines is vertical, the usual math won't work: check for intercept and see
        // if part of segment
        if (other.getDeltaX() == 0) {
            double thisOffset =
                    this.getStartPoint().getY()
                            - (Math.tan(this.getDirectionRadians())) * this.getStartPoint().getX();

            double intercept =
                    (Math.tan(this.getDirectionRadians()) * other.getStartPoint().getX())
                            + thisOffset;

            return (other.pointOn(new Vector2D(other.getStartPoint().getX(), intercept))
                    && this.pointOn(new Vector2D(other.getStartPoint().getX(), intercept)));
        }
        if (this.getDeltaX() == 0) {
            double otherOffset =
                    other.getStartPoint().getY()
                            - (Math.tan(other.getDirectionRadians()))
                                    * other.getStartPoint().getX();

            double intercept =
                    (Math.tan(other.getDirectionRadians()) * this.getStartPoint().getX())
                            + otherOffset;

            return (this.pointOn(new Vector2D(this.getStartPoint().getX(), intercept))
                    && other.pointOn(new Vector2D(this.getStartPoint().getX(), intercept)));
        }

        // convert from point-slope to y = mx + b

        double thisOffset =
                this.getStartPoint().getY()
                        - (Math.tan(this.getDirectionRadians()) * this.getStartPoint().getX());
        double otherOffset =
                other.getStartPoint().getY()
                        - (Math.tan(other.getDirectionRadians()) * other.getStartPoint().getX());

        // find coordinates of possible intersection

        double xCoord =
                (otherOffset - thisOffset)
                        / (Math.tan(this.getDirectionRadians())
                                - Math.tan(other.getDirectionRadians()));
        double yCoord = (Math.tan(this.getDirectionRadians()) * xCoord) + thisOffset;

        // return whether the intersection is on both lines
        return (this.pointOn(new Vector2D(xCoord, yCoord))
                && other.pointOn(new Vector2D(xCoord, yCoord)));
    }

    /**
     * Returns the startpoint of the line.
     *
     * @return A vector with endpoint representing the startpoint of this line.
     */
    public Vector2D getStartPoint() {
        return startPoint;
    }

    /**
     * Returns the endpoint of the line.
     *
     * @return A vector with endpoint representing the endpoint of this line.
     */
    public Vector2D getEndPoint() {
        return endPoint;
    }
}
