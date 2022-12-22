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

/** Special case of a Vector3D in two dimensions. */
public class Vector2D extends Vector3D {
    /**
     * Constructs a 2-dimensional vector.
     *
     * @param x The X (i-hat, positive to the horizontal right on a graph) component of the vector.
     * @param y The Y (j-hat, positive up vertically on a graph) component of the vector.
     */
    public Vector2D(double x, double y) {
        super(x, y, 0);
    }

    @Override
    /** {@inheritDoc} */
    public double getZ() {
        return 0;
    }
}
