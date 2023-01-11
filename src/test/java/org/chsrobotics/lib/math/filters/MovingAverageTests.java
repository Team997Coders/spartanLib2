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
package org.chsrobotics.lib.math.filters;

import static org.junit.Assert.assertEquals;

import org.chsrobotics.lib.math.filters.MovingAverageFilter.MEAN_IMPLEMENTATION;
import org.junit.Test;

public class MovingAverageTests {
    private final double epsilon = 0.0001;

    @Test
    public void MovingAverageFilterWorksInfiniteWindow() {
        MovingAverageFilter filter = new MovingAverageFilter(0, MEAN_IMPLEMENTATION.ARITHMETIC);

        assertEquals(1, filter.calculate(1), epsilon);
        assertEquals(1, filter.calculate(1), epsilon);
        assertEquals(2, filter.calculate(4), epsilon);
        assertEquals(0, filter.calculate(-6), epsilon);
    }

    @Test
    public void MovingAverageFilterWorksFiniteWindow() {
        MovingAverageFilter filter = new MovingAverageFilter(3, MEAN_IMPLEMENTATION.ARITHMETIC);

        assertEquals(3, filter.calculate(3), epsilon);
        assertEquals(1.5, filter.calculate(0), epsilon);
        assertEquals(0, filter.calculate(-3), epsilon);

        assertEquals(-3, filter.calculate(-6), epsilon);
        assertEquals(-3, filter.calculate(0), epsilon);
        assertEquals(-1, filter.calculate(3), epsilon);
    }
}
