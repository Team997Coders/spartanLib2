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

import org.junit.Test;

public class ExponentialMovingAverageTests {
    private static final double epsilon = 0.0001;

    @Test
    public void exponentialMovingAverageWorks() {
        ExponentialMovingAverage movingAverage = new ExponentialMovingAverage(0.5);

        assertEquals(0.5, movingAverage.calculate(1), epsilon);
        assertEquals(0.75, movingAverage.calculate(1), epsilon);
        assertEquals(2.875, movingAverage.calculate(5), epsilon);
        assertEquals(1.4375, movingAverage.calculate(0), epsilon);
    }
}
