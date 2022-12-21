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
     * Adds the value to the window and calculates the current output of the filter. If dt would be
     * a required parameter for the filter, uses 20 milliseconds (the robot loop period).
     *
     * @param value The value to input to the filter.
     * @return The current output of the filter.
     */
    public abstract double calculate(double value);

    /**
     * Adds the value to the window and calculates the current output of the filter, with a change
     * in time since the last call of this.
     *
     * <p>Some filters do not use time in their calculations, and this method is identical to {@code
     * calculate()} for them.
     *
     * @param value the value to input to the filter.
     * @param dtSeconds The change in time since the last call of the filter.
     * @return The current output of the filter.
     */
    public abstract double calculate(double value, double dtSeconds);

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
     * @param filterA The first filter to sum.
     * @param filterB The other filter to sum.
     * @return A new, composed filter.
     */
    public static final Filter add(Filter filterA, Filter filterB) {
        class AddedFilter extends Filter {
            @Override
            public double calculate(double value) {
                return filterA.calculate(value) + filterB.calculate(value);
            }

            @Override
            public double calculate(double value, double dtSeconds) {
                return filterA.calculate(value, dtSeconds) + filterB.calculate(value, dtSeconds);
            }

            @Override
            public void reset() {
                filterA.reset();
                filterB.reset();
            }

            @Override
            public double getCurrentOutput() {
                return filterA.getCurrentOutput() + filterB.getCurrentOutput();
            }
        }

        return new AddedFilter();
    }

    /**
     * Returns a new filter of the outputs of a filter multiplied by a scalar.
     *
     * @param filter The filter to multiply.
     * @param scalar The scalar value to multiply by.
     * @return A new filter.
     */
    public static Filter scalarMultiply(Filter filter, double scalar) {
        class MultipliedFilter extends Filter {

            @Override
            public double calculate(double value) {
                return filter.calculate(value) * scalar;
            }

            @Override
            public double calculate(double value, double dtSeconds) {
                return filter.calculate(value, dtSeconds) * scalar;
            }

            @Override
            public void reset() {
                filter.reset();
            }

            @Override
            public double getCurrentOutput() {
                return filter.getCurrentOutput() * scalar;
            }
        }
        return new MultipliedFilter();
    }
}
