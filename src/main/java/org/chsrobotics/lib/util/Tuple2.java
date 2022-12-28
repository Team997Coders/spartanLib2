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

import java.util.ArrayList;
import java.util.List;

/**
 * Very simple utility class which can hold any two objects of the same type in order.
 *
 * @param <T> The data type of objects to store.
 */
public class Tuple2<T> {
    private final T valueA;
    private final T valueB;

    /**
     * Constructs a Tuple2.
     *
     * @param valueA First value of T.
     * @param valueB Second value of T.
     */
    public Tuple2(T valueA, T valueB) {
        this.valueA = valueA;
        this.valueB = valueB;
    }

    /**
     * Returns the data of this object as a list of two elements.
     *
     * @return A list, containing two values, of the contents of this instance.
     */
    public List<T> toList() {
        ArrayList<T> list = new ArrayList<>();
        list.add(valueA);
        list.add(valueB);

        return list;
    }

    /**
     * Returns the first value of the tuple.
     *
     * @return An instance of T.
     */
    public T firstValue() {
        return valueA;
    }

    /**
     * Returns the seond value of the tuple.
     *
     * @return An instance of T.
     */
    public T secondValue() {
        return valueB;
    }

    /**
     * Constructs and returns a Tuple2 of two values of the same data type.
     *
     * @param <U> The data type of the values.
     * @param valueA The first instance of U.
     * @param valueB The first instance of U.
     * @return A Tuple2 of the two values.
     */
    public static <U> Tuple2<U> of(U valueA, U valueB) {
        return new Tuple2<U>(valueA, valueB);
    }

    @Override
    public String toString() {
        return "Tuple2 of: " + valueA + ", " + valueB;
    }
}
