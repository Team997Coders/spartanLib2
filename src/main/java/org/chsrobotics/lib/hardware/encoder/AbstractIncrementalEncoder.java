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
package org.chsrobotics.lib.hardware.encoder;

import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public abstract class AbstractIncrementalEncoder extends AbstractEncoder {
    private final DifferentiatingFilter velocityFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter accelerationFilter = new DifferentiatingFilter();

    public AbstractIncrementalEncoder() {
        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public abstract double getUnitsPerCount();

    public abstract boolean getInverted();

    public abstract double getRawCounts();

    @Override
    public double getRawPosition() {
        return getRawCounts();
    }

    @Override
    public double getRawVelocity() {
        return velocityFilter.getCurrentOutput();
    }

    @Override
    public double getRawAcceleration() {
        return accelerationFilter.getCurrentOutput();
    }

    @Override
    public double getConvertedPosition() {
        return unitConversion(getRawPosition());
    }

    @Override
    public double getConvertedVelocity() {
        return unitConversion(getRawVelocity());
    }

    @Override
    public double getConvertedAcceleration() {
        return unitConversion(getRawAcceleration());
    }

    private void periodic(double dtSeconds) {
        velocityFilter.calculate(getRawPosition(), dtSeconds);

        accelerationFilter.calculate(velocityFilter.getCurrentOutput(), dtSeconds);
    }

    private double unitConversion(double in) {
        return in * getUnitsPerCount() * (getInverted() ? -1 : 1);
    }
}
