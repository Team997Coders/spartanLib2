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
import static org.junit.Assert.assertThrows;

import java.security.InvalidParameterException;
import java.util.TreeMap;
import org.junit.Before;
import org.junit.Test;

/** Tests for the MultiPointInterpolator to ensure all the logic works correctly. */
public class MultiPointInterpolatorTests {
    private static final double epsilon = 0.0001;

    private TreeMap<Double, Double> treeMap;
    private MultiPointInterpolator interpolator;

    @Before
    public void setup() { // assigns the treeMap to an empty TreeMap before each test is run.
        treeMap = new TreeMap<Double, Double>();
    }

    @Test
    public void rejectsEmptyMap() {
        assertThrows(
                InvalidParameterException.class,
                () -> {
                    new MultiPointInterpolator(treeMap);
                });
    }

    @Test
    public void rejectsSampleBelowZero() throws Exception {
        treeMap.put(1.0, 1.0);
        interpolator = new MultiPointInterpolator(treeMap);

        assertThrows(
                InvalidParameterException.class,
                () -> {
                    interpolator.sample(-0.5);
                });
    }

    @Test
    public void returnsDefinedValue() throws Exception {
        treeMap.put(0.5, 1.5);
        interpolator = new MultiPointInterpolator(treeMap);

        assertEquals(1.5, interpolator.sample(0.5), epsilon);

        interpolator.putNewPair(3, 2);
        assertEquals(2, interpolator.sample(3), epsilon);
    }

    @Test
    public void handlesDefinedValueAtZero() throws Exception {
        treeMap.put(0.0, 10.0);
        interpolator = new MultiPointInterpolator(treeMap);

        assertEquals(10, interpolator.sample(0), epsilon);
    }

    @Test
    public void interpolates() throws Exception {
        treeMap.put(1.0, 0.0);
        treeMap.put(2.0, 1.0);
        interpolator = new MultiPointInterpolator(treeMap);

        assertEquals(0.5, interpolator.sample(1.5), epsilon);
    }

    @Test
    public void handlesUndefinedZero() throws Exception {
        treeMap.put(1.0, 2.0);
        interpolator = new MultiPointInterpolator(treeMap);

        assertEquals(0.0, interpolator.sample(0), epsilon);
        assertEquals(1.0, interpolator.sample(0.5), epsilon);
    }

    @Test
    public void handlesValueAboveHighestDefined() throws Exception {
        treeMap.put(1.0, 2.0);
        interpolator = new MultiPointInterpolator(treeMap);

        assertEquals(0.0, interpolator.sample(5), epsilon);
    }
}
