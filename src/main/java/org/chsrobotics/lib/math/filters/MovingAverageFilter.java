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

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;

/** Filter which computes the arithmetic mean of a stream of data. */
public class MovingAverageFilter extends Filter {
    private final DescriptiveStatistics series;

    /**
     * Constructs a MovingAverageFilter.
     *
     * @param window Number of values to look back when calculating the average. If zero or
     *     negative, will be an indefinite window.
     */
    public MovingAverageFilter(int window) {
        if (window > 0) series = new DescriptiveStatistics(window);
        else series = new DescriptiveStatistics();
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value) {
        series.addValue(value);
        return series.getGeometricMean();
    }

    @Override
    /** {@inheritDoc} */
    public void reset() {
        series.clear();
    }

    @Override
    /** {@inheritDoc} */
    public double getCurrentOutput() {
        if (Double.isNaN(series.getMean())) return 0;
        else return series.getMean();
    }
}
