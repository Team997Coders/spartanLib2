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
package org.chsrobotics.lib.math;

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

    private void testDataSet(PeakDetectionFilter filter, double[] inputs, int[] expectedOutputs) {
        for (int i = 0; i < inputs.length; i++) {
            double o = filter.calculate(inputs[i]);
            // System.out.print(o + ", ");
            assertEquals(expectedOutputs[i], o, epsilon);
        }
    }

    @Test
    public void PeakDetectionFilterRejectsInvalidWindow() {
        assertThrows(
                InvalidParameterException.class,
                () -> {
                    new PeakDetectionFilter(0, 1, 1, 1);
                });
    }

    @Test
    public void PeakDetectionFilterSensiblyFiltersStaticVarianceSeries() {
        PeakDetectionFilter filter = new PeakDetectionFilter(5, 5, 0.3, 0.3, 0.1);

        double[] input = {
            0, 0.1, 0.2, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0.5, 0.3, 0.2, -0.1, -0.1, -0.25, -0.5, -0.4,
            -0.6, -0.2, -0.1, 0, -0.1, 0, -0.1, 0, 0.1, 0, 0, 0.2, 0.2, -0.25
        };

        int[] output = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            1, 0, -1
        };

        testDataSet(filter, input, output);
    }

    @Test
    public void PeakDetectionFilterSensiblyFiltersGrowingVarianceSeries() {
        PeakDetectionFilter filter = new PeakDetectionFilter(5, 3, 0.1, 0.01, 0.2);

        double[] input = {
            1, 1, 1.1, 1, 0.9, 1, 1, 1.1, 1, 0.9, 1, 1.1, 1, 1, 0.9, 1, 1, 1.1, 1, 1, 1, 1, 1.1,
            0.9, 1, 1.1, 1, 1, 0.9, 1, 1.1, 1, 1, 1.1, 1, 0.8, 0.9, 1, 1.2, 0.9, 1, 1, 1.1, 1.2, 1,
            1.5, 1, 3, 2, 5, 3, 2, 1, 1, 1, 0.9, 1, 1, 3, 2.6, 4, 3, 3.2, 2, 1, 1, 0.8, 4, 4, 2,
            2.5, 1, 1, 1
        };

        int[] expectedOutput = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 1, 0, 0, -1, 1, 1, 1, 1, 0, 0, 0
        };

        testDataSet(filter, input, expectedOutput);
    }

    @Test
    public void PeakDetectionFilterCanFilterNonstationarySeries() {
        PeakDetectionFilter filter = new PeakDetectionFilter(10, 3, 0.25, 0.05);

        double[] input = {
            0, 1, 3, 0, 2, -1, -1, 0, 2, -1, 2, -1, 0, 0, 0.5, 3, -2, -1, -0.5, 2, 4, 3, 0.5, 2, -1,
            -3, 1, -3, 2, 3, -1, -2, -3, -1, 0, 0, -4, 0, 0, -5, -5, 0, 1, 2, -0.5, -2, -2, -5, -3,
            -2, -2, -2, -3, -2, -1, -1, -2, -3, -2, 0, 1, 0, 0, -0.5, 1, -2, -2, -3, -2, -4, -4, -6
        };

        int[] output = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, -1, 0, 0, 0, 1, -1, -1, 0, 1, 1, 1, 0, 0, 0, -1, 0,
            -1, 0, 1, 0, -1, -1, 0, 0, 0, -1, 0, 0, -1, -1, 0, 0, 1, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0,
            0, 0, 0, -1, -1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, -1, -1, -1
        };

        testDataSet(filter, input, output);
    }

    @Test
    public void PeakDetectionFilterProperlyHandlesConstantSeries() {
        PeakDetectionFilter filter = new PeakDetectionFilter(5, 3, 0.1, 0.1);

        double[] input = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        };

        int[] output = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        };

        testDataSet(filter, input, output);
    }

    @Test
    public void PeakDetectionFilterSamplesOfTheFirstWindowReturnZero() {
        PeakDetectionFilter filter = new PeakDetectionFilter(5, 0.1, 1, 1);
        assertEquals(0, filter.calculate(0), epsilon);
        assertEquals(0, filter.calculate(0), epsilon);
        assertEquals(0, filter.calculate(-1), epsilon);
        assertEquals(0, filter.calculate(10), epsilon);
        assertEquals(1, filter.calculate(15), epsilon);
    }
}
