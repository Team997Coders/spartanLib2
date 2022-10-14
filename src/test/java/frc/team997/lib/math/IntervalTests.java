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

import static org.junit.Assert.assertEquals;

import frc.team997.lib.math.Interval.Inclusion;
import org.junit.Test;

/** Tests for the Interval class to make sure all logic works properly */
public class IntervalTests {
    @Test
    public void IntervalInclusionWorksProperly() {
        Interval testInterval = new Interval(Inclusion.INCLUSIVE, 0.0, 1.0, Inclusion.NONINCLUSIVE);
        assertEquals(true, testInterval.includes(0));
        assertEquals(false, testInterval.includes(1));
    }

    @Test
    public void IntervalFlippingWorksProperly() {
        Interval testInterval = new Interval(Inclusion.INCLUSIVE, 5.0, 1.5, Inclusion.NONINCLUSIVE);
        assertEquals(true, testInterval.includes(2));
        assertEquals(false, testInterval.includes(1.5));
    }

    @Test
    public void IntervalInfiniteLimitsWorkProperly() {
        Interval testInterval =
                new Interval(
                        Inclusion.NONINCLUSIVE,
                        0.0,
                        Double.POSITIVE_INFINITY,
                        Inclusion.NONINCLUSIVE);
        assertEquals(true, testInterval.includes(1000000000000.0));
        assertEquals(false, testInterval.includes(0));
    }
}
