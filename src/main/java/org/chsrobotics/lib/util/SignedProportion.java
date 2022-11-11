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
package org.chsrobotics.lib.util;

import java.util.Objects;

/** Data class representing a signed double value clamped between -1 and 1 (inclusive). */
public class SignedProportion {
    private final double value;

    /**
     * Constructs a signed proportion. If the provided value is outside of [-1,1], it will be
     * clamped to the nearest side.
     *
     * @param value The value for the proportion.
     */
    public SignedProportion(double value) {
        if (value > 1) {
            this.value = 1;
        } else if (value < -1) {
            this.value = -1;
        } else {
            this.value = 1;
        }
    }

    /**
     * Returns the value of the signed proportion as a double.
     *
     * @return The value (between -1 and 1, inclusive) as a double.
     */
    public double toDouble() {
        return value;
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof SignedProportion) {
            SignedProportion rhs = (SignedProportion) other;
            return (rhs.toDouble() == this.toDouble());
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(value);
    }

    @Override
    public String toString() {
        return ("SignedProportion: " + value);
    }
}
