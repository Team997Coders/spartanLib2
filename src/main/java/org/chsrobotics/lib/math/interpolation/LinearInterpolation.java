/**
Copyright 2023 FRC Team 997

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
package org.chsrobotics.lib.math.interpolation;

import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.math.geometry.Vector3D;
import org.chsrobotics.lib.util.Sampleable;

/** TODO */
public class LinearInterpolation implements Sampleable<Vector3D> {
    private final Vector3D[] points;

    private final double minReference;
    private final double maxReference;

    /**
     * @param points
     */
    public LinearInterpolation(Vector3D... points) {
        this.points = points;

        Vector3D tMinRef = points[0];
        Vector3D tMaxRef = points[0];

        for (Vector3D point : points) {
            if (point.getX() < tMinRef.getX()) tMinRef = point;
            if (point.getX() > tMaxRef.getX()) tMaxRef = point;
        }

        minReference = tMinRef.getX();
        maxReference = tMaxRef.getX();
    }

    @Override
    /** {@inheritDoc} */
    public double getMinReference() {
        return minReference;
    }

    @Override
    /** {@inheritDoc} */
    public double getMaxReference() {
        return maxReference;
    }

    @Override
    /** */
    public Vector3D sample(double reference) {
        Vector3D floor = null;
        Vector3D ceiling = null;

        double floorX = Double.NEGATIVE_INFINITY;
        double ceilingX = Double.POSITIVE_INFINITY;

        // loop through points, find largest lesser point and smallest greater point
        for (Vector3D point : points) {
            if (point.getX() <= reference && point.getX() > floorX) {
                floor = point;
                floorX = point.getX();
            }
            if (point.getX() >= reference && point.getX() < ceilingX) {
                ceiling = point;
                ceilingX = point.getX();
            }
        }

        // catch edge cases
        if (floor == null) {
            return ceiling;
        } else if (ceiling == null) {
            return floor;
        } else if (reference == floorX && reference == ceilingX && !ceiling.equals(floor)) {
            return null;
        } else if (reference == floorX) {
            return floor;
        } else if (reference == ceilingX) {
            return ceiling;
        }

        // div by 0 unreachable
        double scaledReference = (reference - floorX) / (ceilingX - floorX);

        Vector3D vec = UtilityMath.linearInterpolation(floor, ceiling, scaledReference);

        return new Vector3D(reference, vec.getY(), vec.getZ());
    }
}
