/**
Copyright 2023 FRC Team 997

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
package org.chsrobotics.lib.hardware.base.encoder;

import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

// TODO finish docs

public abstract class AbstractAbsoluteEncoder extends AbstractEncoder {
    private final DifferentiatingFilter convVelocityFilter = new DifferentiatingFilter(true);

    public AbstractAbsoluteEncoder() {
        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public abstract double getOffset();

    public abstract double getUnoffsetConvertedPosition();

    @Override
    public double getConvertedPosition() {
        return UtilityMath.normalizeAngleRadians(getUnoffsetConvertedPosition() + getOffset());
    }

    /**
     * Returns the converted velocity of the encoder.
     *
     * <p>Note that this is able to compensate for "angle wrap" in absolute encoders, by assuming
     * that the shortest distance between two angles is the path taken.
     *
     * <p>{@code getRawVelocity()} does *not* compensate for angle wrap in this way.
     *
     * @return The velocity, in radians per second, of the encoder.
     */
    @Override
    public double getConvertedVelocity() {
        return convVelocityFilter.getCurrentOutput();
    }

    /**
     * Returns the unconverted velocity of the encoder.
     *
     * <p>Note that for absolute encoders, this number is much less useful, as it fails to account
     * for "angle wrap" (e.g. going from 359 degrees to 1 degree counting as a delta of 358, not 2).
     *
     * <p>However, {@code getConvertedVelocity()} is able to account for angle wrap.
     *
     * @return The velocity, in sensor units per second, of the encoder.
     */
    @Override
    public double getRawVelocity() {
        return super.getRawVelocity();
    }

    public double getOffsetToZeroCurrentPosition() {
        return -getUnoffsetConvertedPosition();
    }

    private void periodic(double dtSeconds) {
        convVelocityFilter.calculate(getConvertedPosition(), dtSeconds);
    }
}
