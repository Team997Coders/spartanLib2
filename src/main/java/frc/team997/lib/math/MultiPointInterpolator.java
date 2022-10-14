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

import java.security.InvalidParameterException;
import java.util.TreeMap;

/** A class to provide linear interpolation, using a set of key/value pairs. */
public class MultiPointInterpolator {
    private TreeMap<Double, Double> m_points;

    /**
     * Constructs a MultiPointInterpolator.
     *
     * @param points A map of doubles, with the first term representing the independent variable
     *     (time etc.), and the second the value.
     * @throws InvalidParameterException If the map is empty.
     */
    public MultiPointInterpolator(TreeMap<Double, Double> points) throws Exception {
        if (points.isEmpty()) {
            throw new InvalidParameterException("Length of map must not be zero!");
        }
        m_points = points;
        if (!m_points.containsKey(0.0)) {
            m_points.put(
                    0.0, 0.0); // prevent case where there's no starting value to interpolate from
        }
    }

    /**
     * Adds to the map at runtime.
     *
     * @param points A TreeMap, in the same format as the one used for construction.
     */
    public void putNewPairs(TreeMap<Double, Double> points) {
        m_points.putAll(points);
    }

    /**
     * Adds a new key/value pair to to the map at runtime.
     *
     * @param key Double value of the key.
     * @param value Double value of the... value.
     */
    public void putNewPair(double key, double value) {
        m_points.put(key, value);
    }

    /**
     * Samples the profile for a value at a given key.
     *
     * @param key A double representing the key to sample.
     * @return The value at that place, linearly interpolated if not defined.
     * @throws InvalidParameterException If the key to sample is less than zero.
     */
    public double sample(double key) throws Exception {
        if (key < 0) {
            throw new InvalidParameterException("Place to sample must not be less than zero!");
        }
        if (m_points.containsKey(key)) {
            return m_points.get(key); // return if already defined
        } else {
            Double ceiling = m_points.ceilingKey(key);
            if (ceiling == null) {
                return 0; // if key after highest defined value, return 0
            }
            double floor = m_points.floorKey(key); // interpolate otherwise
            return UtilityMath.simpleLinearInterpolation(
                    m_points.get(floor), floor, m_points.get(ceiling), ceiling, key);
        }
    }
}
