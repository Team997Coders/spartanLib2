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

/** Special case of a Vector3D in one dimension. */
public class Vector1D extends Vector2D {
    /**
     * Constructs a 1-dimensional vector.
     *
     * @param x The X (i-hat, positive to the horizontal right on a graph) component of the vector.
     */
    public Vector1D(double x) {
        super(x, 0);
    }

    @Override
    /** {@inheritDoc} */
    public double getY() {
        return 0;
    }
}
