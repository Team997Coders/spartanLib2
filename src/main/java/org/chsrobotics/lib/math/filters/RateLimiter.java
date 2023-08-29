/**
Copyright 2022-2023 FRC Team 997

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

import edu.wpi.first.math.MathUtil;

/**
 * Filter which constrains the maximum rate of change of a reference. Useful for ensuring that an
 * open-loop-controlled mechanism doesn't accelerate uncontrollably, or that measurements can't
 * change unreasonably fast.
 */
public class RateLimiter extends Filter {
    private final double rateLimit;

    private double lastValue = 0;

    /**
     * Constructs a RateLimiter.
     *
     * @param rateLimit Maximum rate-of-change of the reference, in units per second. If equal to
     *     zero, this will not apply any kind of rate limiting.
     */
    public RateLimiter(double rateLimit) {
        this.rateLimit = rateLimit;
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value, double dtSeconds) {
        double delta = value - lastValue;

        if (rateLimit != 0)
            lastValue =
                    lastValue
                            + MathUtil.clamp(delta, -rateLimit * dtSeconds, rateLimit * dtSeconds);
        else lastValue = value;

        return lastValue;
    }

    @Override
    /** {@inheritDoc} */
    public void reset() {
        lastValue = 0;
    }

    @Override
    /** {@inheritDoc} */
    public double getCurrentOutput() {
        return lastValue;
    }
}
