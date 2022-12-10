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
package org.chsrobotics.lib.math.filters;

/** Common superclass for this library's filters. */
public abstract class Filter {
    /**
     * Adds the value to the window and calculates the current output of the filter
     *
     * @param value The value to input to the filter.
     * @return The current output of the filter.
     */
    public abstract double calculate(double value);

    /** Resets the history of the filter. */
    public abstract void reset();

    /**
     * Returns the current output of the filter without updating with a new value.
     *
     * @return The current output of the filter (0 if no values have been given to {@code
     *     calculate()}).
     */
    public abstract double getCurrentOutput();

    /**
     * Returns a filter of a sum of the outputs of two other filters.
     *
     * @param other The other filter to sum.
     * @return A new, composed filter.
     */
    public final Filter add(Filter other) {
        class AddedFilter extends Filter {
            @Override
            public double calculate(double value) {
                return this.calculate(value) + other.calculate(value);
            }

            @Override
            public void reset() {
                this.reset();
                other.reset();
            }

            @Override
            public double getCurrentOutput() {
                return this.getCurrentOutput() + other.getCurrentOutput();
            }
        }

        return new AddedFilter();
    }

    /**
     * Returns a new filter of the outputs of this filter multiplied by a scalar.
     *
     * @param scalar The scalar value to multiply by.
     * @return A new filter.
     */
    public final Filter scalarMultiply(double scalar) {
        class MultipliedFilter extends Filter {

            @Override
            public double calculate(double value) {
                return this.calculate(value) * scalar;
            }

            @Override
            public void reset() {
                this.reset();
            }

            @Override
            public double getCurrentOutput() {
                return this.getCurrentOutput() * scalar;
            }
        }
        return new MultipliedFilter();
    }
}
