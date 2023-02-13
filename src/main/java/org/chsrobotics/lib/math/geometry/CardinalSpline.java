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

import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Implementation of a Cardinal spline curve in 3- (or fewer) dimensional space.
 *
 * <p>A Cardinal spline is a special case of a cubic Hermite spline such that the tangent lines at
 * each control point are congruent to some constant scalar times the vector between the immediately
 * preceding and following control points.
 *
 * <p>Cardinal splines are useful because they are guaranteed to pass through all control points.
 * They also have C^0 and C^1 continuity, meaning that there are no discontinuities in their spacial
 * position or the first derivative of their position. However, C^2 and higher continuity is
 * unlikely to occur.
 *
 * <p>Since there is no point preceding the first control point or following the last, and for FRC
 * trajectory following applications the orientation of the robot almost always matters, this
 * implementation allows explicit control over the angle of the starting and ending tangents.
 *
 * <p>More detail on splines can be found from this excellent source:
 *
 * <p><a href="https://www.youtube.com/watch?v=jvPPXbo87ds">The Continuity of Splines</a>
 */
public class CardinalSpline {
    private final double tension;

    private final Vector3D[] points;

    private final Rotation3d startAngle;
    private final Rotation3d endAngle;

    /**
     * Constructs a CardinalSpline.
     *
     * @param tension A scalar to adjust the length of the Hermite tangent lines. Higher numbers
     *     produce higher curvature in the spline. Must be greater than 0, 1 is about the maximum
     *     sensible number. 0.5 generally produces consistent good results.
     * @param startAngle The orientation of the spline at the startpoint. Use the method {@code
     *     UtilityMath.fromRotation2d} to generate compliant Rotation3ds while working in
     *     2-dimensional space.
     * @param endAngle The orientation of the spline at the endpoint. Use the method {@code
     *     UtilityMath.fromRotation2d} to generate compliant Rotation3ds while working in
     *     2-dimensional space.
     * @param points Vectors with endpoints representing the waypoints that the trajectory should
     *     pass through, including the start and endpoints of the spline.
     */
    public CardinalSpline(
            double tension, Rotation3d startAngle, Rotation3d endAngle, Vector3D... points) {
        this.tension = tension;

        this.points = points;

        this.startAngle = startAngle;
        this.endAngle = endAngle;
    }

    /**
     * Returns the position of the spline as a vector with endpoint representing the point.
     *
     * @param reference Place inside the spline to sample. Must be in the range [0, number of
     *     control points] (inclusive).
     * @return The position of the spline. If {@code reference} was less than 0 or greater than the
     *     number of control points, returns {@code null}.
     */
    public Vector3D sample(double reference) {
        if (reference < 0 || reference > points.length - 1) return null;

        Vector3D hermiteStart = points[(int) reference];
        Vector3D hermiteEnd = points[(int) reference + 1];

        Vector3D startTan;
        Vector3D endTan;

        // use the rotation axis vector of the quaternion representation of the start angle for
        // tangents

        if (reference > 1)
            startTan = hermiteEnd.subtract(points[(int) reference - 1]).scalarMultiply(tension);
        else
            startTan =
                    new Vector3D(
                            startAngle.getQuaternion().getX(),
                            startAngle.getQuaternion().getY(),
                            startAngle.getQuaternion().getZ());

        if (reference < points.length - 2)
            endTan = points[(int) reference + 2].subtract(hermiteStart).scalarMultiply(tension);
        else
            endTan =
                    new Vector3D(
                            endAngle.getQuaternion().getX(),
                            endAngle.getQuaternion().getY(),
                            endAngle.getQuaternion().getZ());

        double localReference = reference - ((int) reference);

        Vector3D startControlPoint = points[(int) reference];
        Vector3D endControlPoint = points[(int) reference + 1];

        Vector3D h00 =
                startControlPoint.scalarMultiply(
                        (2 * Math.pow(localReference, 3)) - (3 * Math.pow(localReference, 2)) + 1);

        Vector3D h10 =
                startTan.scalarMultiply(
                        (Math.pow(localReference, 3))
                                - (2 * Math.pow(localReference, 2))
                                + localReference);

        Vector3D h11 =
                endControlPoint.scalarMultiply(
                        (-2 * Math.pow(localReference, 3)) + (3 * Math.pow(localReference, 2)));

        Vector3D h01 =
                endTan.scalarMultiply(Math.pow(localReference, 3) - Math.pow(localReference, 2));

        return h00.add(h10).add(h11).add(h01);
    }
}
