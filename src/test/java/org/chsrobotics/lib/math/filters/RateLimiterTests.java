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

public class RateLimiterTests {
    private final double epsilon = 0.0001;

    @Test
    public void RateLimiterFunctional() {
        RateLimiter limiter = new RateLimiter(5, 1);

        assertEquals(2, limiter.calculate(2), epsilon);
        assertEquals(7, limiter.calculate(10), epsilon);
        assertEquals(2, limiter.calculate(-5), epsilon);
        assertEquals(0, limiter.calculate(0), epsilon);
    }
}
