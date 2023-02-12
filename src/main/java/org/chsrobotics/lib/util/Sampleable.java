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
package org.chsrobotics.lib.util;

/**
 * Interface for a class representing a single-input function with lower and upper domain limits.
 *
 * @param T Type returned by the function.
 */
public interface Sampleable<T> {
    T sample(double reference);

    /**
     * Returns the minimum allowed reference (input) to the function.
     *
     * @return The minimum allowed reference.
     */
    double getMinReference();

    /**
     * Returns the maximum allowed reference (input) to the function.
     *
     * @return The maximum allowed reference.
     */
    double getMaxReference();

    /**
     * Returns whether the given reference is outside the domain of the function.
     *
     * @param reference The reference to check.
     * @return Whether the given reference is safe to input to this implementation's {@code
     *     sample()} method.
     */
    default boolean isOutOfBounds(double reference) {
        return (reference < getMinReference()
                || reference > getMaxReference()
                || Double.isNaN(reference));
    }

    /**
     * Returns the value of the function at the minimum reference.
     *
     * @return The value of the function at the minimum reference.
     */
    default T getStart() {
        return sample(getMinReference());
    }

    /**
     * Returns the value of the function at the maximum reference.
     *
     * @return The value of the function at the maximum reference.
     */
    default T getEnd() {
        return sample(getMaxReference());
    }
}
