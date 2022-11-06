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

/**
 * A filter that returns the rate of change (derivative) of a stream of data.
 *
 * <p>Approximated with finite timesteps.
 */
public class DifferentiatingFilter implements Filter {
    private final double defaultRobotPeriodSeconds = 0.02;
    private double lastValue = 0;
    private double currentDeriv = 0;

    /**
     * Calculates the rate of change (derivative) of a value relative to previous values in input to
     * the filter, using a given change in time.
     *
     * <p>The initial previous value is 0.
     *
     * <p>Returns 0 if {@code dt} is equal to 0 (although negative values of dt won't be meaningful
     * either).
     *
     * @param value The double value to input to the filter.
     * @param dt The elapsed time (in seconds) since the last call of either {@code calculate}
     *     method.
     * @return The rate of change, in units/second, between the current {@code value} and the
     *     previous {@code value}.
     */
    public double calculate(double value, double dt) {
        if (dt == 0) return 0;
        currentDeriv = (value - lastValue) / dt;
        lastValue = value;
        return currentDeriv;
    }

    /**
     * Calculates the rate of change (derivative) of a value relative to previous values in input to
     * the filter, using a the default loop time of the robot.
     *
     * <p>The initial previous value is 0.
     *
     * @param value The double value to input to the filter.
     * @return The rate of change, in units/second, between the current {@code value} and the
     *     previous {@code value}.
     */
    @Override
    public double calculate(double value) {
        return calculate(value, defaultRobotPeriodSeconds);
    }

    @Override
    public void reset() {
        lastValue = 0;
    }

    @Override
    public double getCurrentOutput() {
        return currentDeriv;
    }
}
