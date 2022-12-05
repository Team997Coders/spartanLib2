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

import edu.wpi.first.math.MathUtil;

/**
 * Filter which constrains the maximum rate of change of a reference. Useful for ensuring that an
 * open-loop-controlled mechanism doesn't accelerate uncontrollably, or that measurements can't
 * change unreasonably fast.
 */
public class RateLimiter implements Filter {
    private final double rateLimit;
    private final double dtSeconds;

    private double lastValue = 0;

    /**
     * Constructs a RateLimiter.
     *
     * @param rateLimit Maximum rate-of-change of the reference, in units per second.
     * @param dtSeconds Time, in seconds, expected between calls of this method.
     */
    public RateLimiter(double rateLimit, double dtSeconds) {
        this.rateLimit = rateLimit;
        this.dtSeconds = dtSeconds;
    }

    /**
     * Constructs a RateLimiter with the default robot loop cycle time.
     *
     * @param rateLimit Maximum rate-of-change of the reference, in units per second.
     */
    public RateLimiter(double rateLimit) {
        this(rateLimit, 0.02);
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value) {
        double delta = value - lastValue;

        lastValue =
                lastValue + MathUtil.clamp(delta, -rateLimit * dtSeconds, rateLimit * dtSeconds);

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
