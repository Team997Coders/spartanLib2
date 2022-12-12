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

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class IntegratingFilterTests {
    private final double epsilon = 0.0001;

    @Test
    public void IntegratingFilterWorksInfiniteWindow() {
        IntegratingFilter filter = new IntegratingFilter(0);

        assertEquals(0, filter.calculate(0, 1), epsilon);
        assertEquals(5, filter.calculate(10, 0.5), epsilon);
        assertEquals(-5, filter.calculate(-5, 2), epsilon);
    }

    @Test
    public void IntegratingFilterWorksFiniteWindow() {
        IntegratingFilter filter = new IntegratingFilter(3);

        assertEquals(4, filter.calculate(2, 2), epsilon);
        assertEquals(2, filter.calculate(-4, 0.5), epsilon);
        assertEquals(3, filter.calculate(1, 1), epsilon);

        assertEquals(-1, filter.calculate(0, 1), epsilon);
        assertEquals(2, filter.calculate(1, 1), epsilon);
    }
}
