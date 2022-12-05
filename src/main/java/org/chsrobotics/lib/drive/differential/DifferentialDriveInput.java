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
package org.chsrobotics.lib.drive.differential;

import java.util.Objects;
import org.apache.commons.math3.util.Precision;

/**
 * A data class that holds differential drivetrain inputs, split into drivetrain left and right
 * sides.
 */
public class DifferentialDriveInput {
    public final double left;
    public final double right;

    /**
     * Constructs a DifferentialDriveInput.
     *
     * @param left : value corresponding with left side input.
     * @param right : value corresponding with right side input.
     */
    public DifferentialDriveInput(double left, double right) {
        this.left = left;
        this.right = right;
    }

    /**
     * Returns a new DifferentialDriveInput with the left and right values multiplied by a scalar.
     *
     * @param scalar The multiplicand.
     * @return A scaled DifferentialMove.
     */
    public DifferentialDriveInput multiply(double scalar) {
        return new DifferentialDriveInput(left * scalar, right * scalar);
    }

    /**
     * Returns a new DifferentialMove consisting of the sum of left and right values.
     *
     * @param other The DifferentialMove to add.
     * @return The sum of the two DifferentialMoves.
     */
    public DifferentialDriveInput add(DifferentialDriveInput other) {
        return new DifferentialDriveInput(left + other.left, right + other.right);
    }

    @Override
    public String toString() {
        return "Move[left: " + left + ", right: " + right + "]";
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        double epsilon = 0.000001;
        DifferentialDriveInput that = (DifferentialDriveInput) o;
        return Precision.equals(that.left, left, epsilon)
                && Precision.equals(that.right, right, epsilon);
    }

    @Override
    public int hashCode() {
        return Objects.hash(left, right);
    }
}
