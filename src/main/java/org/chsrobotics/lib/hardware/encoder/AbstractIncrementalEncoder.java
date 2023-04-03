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

import edu.wpi.first.util.datalog.DataLog;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public abstract class AbstractIncrementalEncoder extends AbstractEncoder {
    private final DifferentiatingFilter velocityFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter accelerationFilter = new DifferentiatingFilter();

    private boolean logsConstructed = false;

    private Logger<Double> rawPositionLogger;
    private Logger<Double> rawVelocityLogger;
    private Logger<Double> rawAccelerationLogger;

    private Logger<Double> convertedPositionLogger;
    private Logger<Double> convertedVelocityLogger;
    private Logger<Double> convertedAccelerationLogger;

    public AbstractIncrementalEncoder() {
        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public abstract double getUnitsPerCount();

    public abstract boolean getInverted();

    public abstract double getRawCounts();

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            logsConstructed = true;

            LoggerFactory<Double> factory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            rawPositionLogger = factory.getLogger(name + "/rawPosition_counts");
            rawVelocityLogger = factory.getLogger(name + "/rawVelocity_counts_per_second");
            rawAccelerationLogger =
                    factory.getLogger(name + "/rawAcceleration_counts_per_second_squared");
            convertedPositionLogger = factory.getLogger(name + "/convertedPosition_units");
            convertedVelocityLogger =
                    factory.getLogger(name + "/convertedVelocity_units_per_second");
            convertedAccelerationLogger =
                    factory.getLogger(name + "/convertedAcceleration_units_per_second_squared");

            PeriodicCallbackHandler.registerCallback(this::updateLogs);
        }
    }

    private void updateLogs(double dtSeconds) {
        if (logsConstructed) {
            rawPositionLogger.update(getRawPosition());
            rawVelocityLogger.update(getRawVelocity());
            rawAccelerationLogger.update(getRawAcceleration());

            convertedPositionLogger.update(getConvertedPosition());
            convertedVelocityLogger.update(getConvertedVelocity());
            convertedAccelerationLogger.update(getConvertedAcceleration());
        }
    }

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
