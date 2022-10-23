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

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

import java.security.InvalidParameterException;
import org.junit.Test;

/**
 * Tests for the PeakDetectionFilter.
 *
 * <p>Unfortunately, it's impractical to automate the kind of qualitative tests we want, so this
 * just checks if it behaves as expected over a predefined set of data.
 */
public class PeakDetectionFilterTests {
    private static final double epsilon = 0.0001;

    private static final double[] testInputData = {
        1, 1, 1.1, 1, 0.9, 1, 1, 1.1, 1, 0.9, 1, 1.1, 1, 1, 0.9, 1, 1, 1.1, 1, 1, 1, 1, 1.1, 0.9, 1,
        1.1, 1, 1, 0.9, 1, 1.1, 1, 1, 1.1, 1, 0.8, 0.9, 1, 1.2, 0.9, 1, 1, 1.1, 1.2, 1, 1.5, 1, 3,
        2, 5, 3, 2, 1, 1, 1, 0.9, 1, 1, 3, 2.6, 4, 3, 3.2, 2, 1, 1, 0.8, 4, 4, 2, 2.5, 1, 1, 1
    };

    private static final Integer[] expectedResult = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
        1, 1, 1, 0, 0, -1, 1, 1, 1, 1, 0, 0, 0
    };

    @Test
    public void PeakDetectionFilterRejectsInvalidWindow() {
        assertThrows(
                InvalidParameterException.class,
                () -> {
                    new PeakDetectionFilter(0, 1, 1, 1);
                });
    }

    @Test
    public void PeakDetectionFilterCorrectlyFiltersTestSet() {
        PeakDetectionFilter filter = new PeakDetectionFilter(5, 3, 0.1, 0.01, 0.2);

        for (int i = 0; i < testInputData.length; i++) {
            assertEquals(expectedResult[i], Integer.valueOf(filter.calculate(testInputData[i])));
        }
    }

    @Test
    public void SamplesOfTheFirstWindowReturnZero() {
        PeakDetectionFilter filter = new PeakDetectionFilter(5, 0.1, 1, 1);
        assertEquals(0, filter.calculate(0));
        assertEquals(0, filter.calculate(0));
        assertEquals(0, filter.calculate(-1));
        assertEquals(0, filter.calculate(10));
        assertEquals(1, filter.calculate(15));
    }

    @Test
    public void GetSeriesInWindowReturnsProperly() {
        PeakDetectionFilter filter = new PeakDetectionFilter(3, 1, 0.5, 0.5);
        double[] expectedOutput = {0, 0, 5};
        filter.calculate(0);
        filter.calculate(5);
        assertArrayEquals(expectedOutput, filter.getSeriesInWindow(), epsilon);
    }
}
