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
package org.chsrobotics.lib.trajectory.planning;

/**
 * Interface implementing a cost function for path planning algorithms, which returns the numerical
 * cost of traversing from a node holding a certain value to another.
 *
 * <p>Note to users: make sure your costs are positive (except in a few cases)!
 *
 * @param <T> Data type of the values.
 */
public interface CostFunction<T> {

    /**
     * Returns the cost of moving between nodes containing the given values.
     *
     * @param valueA The first given value.
     * @param valueB The second given value.
     * @return The numerical cost of moving between nodes.
     */
    double evaluate(T valueA, T valueB);
}
