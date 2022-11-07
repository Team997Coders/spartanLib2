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
package frc.team997.lib.math;

/** Common interface for this library's filters. */
public interface Filter {
    /**
     * Adds the value to the window and calculates the current output of the filter
     *
     * @param value The value to input to the filter.
     * @return The current output of the filter (0 if no values have been given to {@code
     *     calculate()}).
     */
    double calculate(double value);

    /** Resets the history of the filter. */
    void reset();

    /**
     * Returns the current output of the filter without updating with a new value.
     *
     * @return The current output of the filter (0 if no values have been given to {@code
     *     calculate()}).
     */
    double getCurrentOutput();
}
