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

/** Tests for the ThresholdFilter. */
public class ThresholdFilterTests {
    @Test
    public void ThresholdFilterWorksForwards() {
        ThresholdFilter filterA = new ThresholdFilter(0, false);
        assertEquals(-1, filterA.calculate(-1), 0);
        assertEquals(0, filterA.calculate(1), 0);
        assertEquals(0, filterA.calculate(0), 0);

        ThresholdFilter filterB = new ThresholdFilter(0.2, false);
        assertEquals(0.1, filterB.calculate(0.1), 0);
        assertEquals(0, filterB.calculate(98.1), 0);
        assertEquals(0.2, filterB.calculate(0.2), 0);
    }

    @Test
    public void ThresholdFilterWorksBackwards() {
        ThresholdFilter filterA = new ThresholdFilter(0, true);
        assertEquals(0, filterA.calculate(-1), 0);
        assertEquals(1, filterA.calculate(1), 0);

        ThresholdFilter filterB = new ThresholdFilter(0.2, true);
        assertEquals(0, filterB.calculate(0.1), 0);
        assertEquals(98.1, filterB.calculate(98.1), 0);
        assertEquals(0.2, filterB.calculate(0.2), 0);
    }
}
