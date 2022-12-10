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

import java.security.InvalidParameterException;
import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;

/**
 * A filter to isolate peaks in a stream of data.
 *
 * <p>Returns 1 if a given value is greater than a rolling average of standard deviations away from
 * the median, -1 if it's smaller by that amount or more.
 *
 * <p>Returns 0 for the first {@code window - 1} inputs.
 *
 * <p>Concept from the Stack Overflow answer cited below.
 *
 * <p>Generally when tuning, there's a balance to be made between window (number of past data points
 * to consider) and stddev and mean (influence a value considered a peak will have on the total mean
 * and standard deviation of the data) influences. A large window will result in more phase lag, but
 * can allow for influences of 1.0 for stddev and mean, as signals will be surrounded by much more
 * data. However, a small window has little phase lag, but can be susceptible to high influences
 * from peaks (they are, after all, outliers by definition). Larger windows are almost always
 * recommended if you can ignore the slight phase lag issue.
 *
 * <p>Special care should be taken when the series exhibits trends over time to balance influences
 * so as not to completely erase or amplify said trend.
 *
 * <p>Brakel, J.P.G. van (2014). "Robust peak detection algorithm using z-scores". Stack Overflow.
 * Available at:
 * https://stackoverflow.com/questions/22583391/peak-signal-detection-in-realtime-timeseries-data/22640362#22640362
 * (version: 2020-11-08).
 */
public class PeakDetectionFilter extends Filter {
    private final double threshold;
    private final double standardDeviationInfluence;
    private final double meanInfluence;
    private final double minimumDelta;
    private final DescriptiveStatistics standardDeviationSeries;
    private final DescriptiveStatistics meanSeries;
    private final DescriptiveStatistics series;

    private int returnValue = 0;

    /**
     * Constructs a PeakDetection filter with a minimum delta from the mean to be considered a
     * signal.
     *
     * @param window The number of past data points to consider for mean and stddev calculations.
     * @param threshold The number of standard deviations above or below the mean needed to be
     *     considered a signal.
     * @param standardDeviationInfluence The effect a signal will have on the standard deviation, in
     *     [0,1].
     * @param meanInfluence The effect a signal will have on the mean, in [0,1].
     * @param minimumDelta The minimum distance from the mean requrired to be considered
     *     significant. An input of zero means that any variance above {@code threshold} standard
     *     deviations will be counted, no matter how small.
     * @throws InvalidParameterException If the window is less than 2.
     */
    public PeakDetectionFilter(
            int window,
            double threshold,
            double standardDeviationInfluence,
            double meanInfluence,
            double minimumDelta) {
        if (window < 2) {
            throw new InvalidParameterException(
                    "Window of PeakDetectionFilter must be greater than 1 for meaningful answers!");
        }
        this.threshold = threshold;
        this.standardDeviationInfluence = standardDeviationInfluence;
        this.meanInfluence = meanInfluence;
        this.minimumDelta = minimumDelta;
        standardDeviationSeries = new DescriptiveStatistics(window);
        meanSeries = new DescriptiveStatistics(window);
        series = new DescriptiveStatistics(window);
    }

    /**
     * Constructs a PeakDetection filter.
     *
     * @param window The number of past data points to consider for mean and stddev calculations.
     * @param threshold The number of standard deviations above or below the mean needed to be
     *     considered a signal.
     * @param standardDeviationInfluence The effect a signal will have on the standard deviation, in
     *     [0,1].
     * @param meanInfluence The effect a signal will have on the mean, in [0,1].
     * @throws InvalidParameterException If the window is less than 2.
     */
    public PeakDetectionFilter(
            int window, double threshold, double standardDeviationInfluence, double meanInfluence) {
        this(window, threshold, standardDeviationInfluence, meanInfluence, 0);
    }

    /**
     * Returns whether a value is considered a peak, a valley, or neither within the combined input
     * data.
     *
     * <p>Returns 0 for the first {@code window-1} inputs.
     *
     * @param value The value to input to the filter.
     * @return 1 if the input value represents a peak, -1 if it represents a valley, 0 if it
     *     represents neither.
     */
    @Override
    public double calculate(double value) {
        if ((int) series.getN() == 0) {
            standardDeviationSeries.addValue(value);
            meanSeries.addValue(value);
            series.addValue(value);
        }
        // prevents exceptions when trying to sample the previous value on the first loop

        if ((int) series.getN() < series.getWindowSize()) {
            returnValue = 0;
            // if there aren't enough data points to fill the window, return 0
        } else if (Math.abs(value - meanSeries.getMean())
                        > (threshold * standardDeviationSeries.getStandardDeviation())
                && (Math.abs(value - meanSeries.getMean()) >= minimumDelta)) {
            if (value > meanSeries.getMean()) {
                returnValue = 1;
            } else {
                returnValue = -1;
            }
            // if the value is greater than *threshold* stddevs away from the mean, return whether
            // it's larger or smaller than the mean
        } else {
            returnValue = 0;
            // else, it's within the threshold and is thus not considered a peak
        }

        meanSeries.addValue(
                (meanInfluence * value)
                        + ((1 - meanInfluence)
                                * meanSeries.getElement((int) meanSeries.getN() - 1)));
        // influence is the proportion of the current value to the previous value in the series,
        // or, how much a single data point is able to change the mean and stddev
        standardDeviationSeries.addValue(
                (standardDeviationInfluence * value)
                        + ((1 - standardDeviationInfluence)
                                * standardDeviationSeries.getElement(
                                        (int) standardDeviationSeries.getN() - 1)));
        series.addValue(value);

        return returnValue;
    }

    /** {@inheritDoc} */
    @Override
    public void reset() {
        standardDeviationSeries.clear();
        meanSeries.clear();
        series.clear();
    }

    /** {@inheritDoc} */
    @Override
    public double getCurrentOutput() {
        return returnValue;
    }
}