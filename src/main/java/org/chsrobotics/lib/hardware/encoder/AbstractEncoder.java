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
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.telemetry.IntrinsicLoggable;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public abstract class AbstractEncoder implements IntrinsicLoggable, StalenessWatchable {
    private final DifferentiatingFilter rawVFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter rawAFilter = new DifferentiatingFilter();

    private final DifferentiatingFilter convVFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter convAFilter = new DifferentiatingFilter();

    private boolean logsConstructed = false;

    private Logger<Double> rawPosLogger;
    private Logger<Double> rawVLogger;
    private Logger<Double> rawALogger;

    private Logger<Double> convPosLogger;
    private Logger<Double> convVLogger;
    private Logger<Double> convALogger;

    private Logger<Boolean> stalenessWatchdogTriggeredLogger;

    private int numStaleCycles = 0;
    private int stalenessThresholdCycles = StalenessWatchable.defaultStalenessThresholdCycles;

    public AbstractEncoder() {
        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public abstract double getRawPosition();

    public double getRawVelocity() {
        return rawVFilter.getCurrentOutput();
    }

    public double getRawAcceleration() {
        return rawAFilter.getCurrentOutput();
    }

    public abstract double getConvertedPosition();

    public double getConvertedVelocity() {
        return convVFilter.getCurrentOutput();
    }

    public double getConvertedAcceleration() {
        return convAFilter.getCurrentOutput();
    }

    private void periodic(double dtSeconds) {
        rawVFilter.calculate(getRawPosition(), dtSeconds);
        rawAFilter.calculate(rawVFilter.getCurrentOutput(), dtSeconds);

        convVFilter.calculate(getConvertedPosition(), dtSeconds);
        convAFilter.calculate(convVFilter.getCurrentOutput(), dtSeconds);

        if (shouldIncrementStalenessCounter()) numStaleCycles++;
        else resetStalenessWatchdog();
    }

    @Override
    public boolean shouldIncrementStalenessCounter() {
        return (getRawVelocity() == 0);
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            LoggerFactory<Double> factory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            rawPosLogger = factory.getLogger(name + "/rawPosition");
            rawVLogger = factory.getLogger(name + "/rawVelocity");
            rawALogger = factory.getLogger(name + "/rawAcceleration");

            convPosLogger = factory.getLogger(name + "/convertedPosition");
            convVLogger = factory.getLogger(name + "/convertedPosition");
            convALogger = factory.getLogger(name + "/convertedAcceleration");

            stalenessWatchdogTriggeredLogger =
                    new Logger<>(
                            log,
                            name + "/stalenessWatchdogTriggered",
                            subdirName,
                            publishToNT,
                            recordInLog);

            PeriodicCallbackHandler.registerCallback(this::updateLogs);

            logsConstructed = true;
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            rawPosLogger.update(getRawPosition());
            rawVLogger.update(getRawVelocity());
            rawALogger.update(getRawAcceleration());

            convPosLogger.update(getConvertedPosition());
            convVLogger.update(getConvertedVelocity());
            convALogger.update(getConvertedAcceleration());

            stalenessWatchdogTriggeredLogger.update(getStalenessWatchdogTriggered());
        }
    }

    @Override
    public boolean getStalenessWatchdogTriggered() {
        return (numStaleCycles >= stalenessThresholdCycles);
    }

    @Override
    public void resetStalenessWatchdog() {
        numStaleCycles = 0;
    }

    @Override
    public void setStalenessThreshold(int cycles) {
        stalenessThresholdCycles = cycles;
    }

    @Override
    public int getStalenessThresholdCycles() {
        return stalenessThresholdCycles;
    }

    @Override
    public int getCurrentStalenessCount() {
        return numStaleCycles;
    }
}
