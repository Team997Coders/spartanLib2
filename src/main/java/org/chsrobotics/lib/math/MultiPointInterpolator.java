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

import java.security.InvalidParameterException;
import java.util.TreeMap;

/**
 * Provides linear interpolation through a map of two variables.
 *
 * <p>This is not a full line- or curve- fitting interpolation, it only considers the two closest
 * values in the map.
 */
public class MultiPointInterpolator {
    private final TreeMap<Double, Double> points;

    /**
     * Constructs a MultiPointInterpolator.
     *
     * @param points A map of doubles, with the first term representing the independent variable
     *     (time etc.), and the second the value.
     * @throws InvalidParameterException If the map is empty.
     */
    public MultiPointInterpolator(TreeMap<Double, Double> points) throws InvalidParameterException {
        if (points.isEmpty()) {
            throw new InvalidParameterException("Length of map must not be zero!");
        }
        this.points = points;
        if (!this.points.containsKey(0.0)) {
            this.points.put(
                    0.0, 0.0); // prevent case where there's no starting value to interpolate from
        }
    }

    /**
     * Adds to the map at runtime.
     *
     * @param points A TreeMap, in the same format as the one used for construction.
     */
    public void putNewPairs(TreeMap<Double, Double> points) {
        this.points.putAll(points);
    }

    /**
     * Adds a new key/value pair to to the map at runtime.
     *
     * @param key Double value of the key.
     * @param value Double value of the... value.
     */
    public void putNewPair(double key, double value) {
        points.put(key, value);
    }

    /**
     * Samples the profile for a value at a given key.
     *
     * @param key A double representing the key to sample.
     * @return The value at that place, linearly interpolated if not already defined by the input
     *     map.
     * @throws InvalidParameterException If the key to sample is less than zero.
     */
    public double sample(double key) throws InvalidParameterException {
        if (key < 0) {
            throw new InvalidParameterException("Place to sample must not be less than zero!");
        }
        if (points.containsKey(key)) {
            return points.get(key); // return if already defined
        } else {
            Double ceiling = points.ceilingKey(key);
            if (ceiling == null) {
                return 0; // if key after highest defined value, return 0
            }
            double floor = points.floorKey(key); // interpolate otherwise
            return UtilityMath.simpleLinearInterpolation(
                    points.get(floor), floor, points.get(ceiling), ceiling, key);
        }
    }
}
