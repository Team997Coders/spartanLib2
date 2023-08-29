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
package org.chsrobotics.lib.hardware.base.powerDistribution;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

// TODO: docs
public class SpartanPDP extends AbstractPowerDistribution {
    public static record PDPConfig(int canID) {
        public static PDPConfig getAutoCanID() {
            return new PDPConfig(0);
        }
    }

    // indexing of channels is 0-based-- 15 is actual highest channel num
    private static final int numChannels = 16;

    private final PDPConfig config;
    private final PowerDistribution pdp;

    private Logger<Double> temperatureCLogger;

    private Logger<Double> totalCurrentALogger;
    private Logger<Double> accumulatedEnergyJLogger;

    private Logger<Double> busVoltageVLogger;

    private final Map<Integer, Logger<Double>> channelCurrentLoggers = new HashMap<>();

    private boolean logsConstructed = false;

    private int stalenessCount = 0;
    private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;
    private double lastBusVoltage = 0;

    public SpartanPDP(PDPConfig config) {
        this.config = config;

        this.pdp = new PowerDistribution(config.canID, ModuleType.kCTRE);

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public PDPConfig getConfig() {
        return config;
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            LoggerFactory<Double> doubleFactory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            for (int i = 0; i < numChannels; i++) {
                channelCurrentLoggers.put(
                        i,
                        doubleFactory.getLogger(
                                name + "/channel" + String.valueOf(i) + "/current_A"));
            }

            temperatureCLogger = doubleFactory.getLogger(name + "/temperature_C");

            totalCurrentALogger = doubleFactory.getLogger(name + "/totalCurrent_A");
            accumulatedEnergyJLogger = doubleFactory.getLogger(name + "/accumulatedEnergy_J");

            busVoltageVLogger = doubleFactory.getLogger(name + "/busVoltage_V");

            logsConstructed = true;

            PeriodicCallbackHandler.registerCallback(this::updateLogs);
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            temperatureCLogger.update(pdp.getTemperature());

            totalCurrentALogger.update(pdp.getTotalCurrent());
            accumulatedEnergyJLogger.update(pdp.getTotalEnergy());

            busVoltageVLogger.update(pdp.getVoltage());

            for (Entry<Integer, Logger<Double>> entry : channelCurrentLoggers.entrySet()) {
                entry.getValue().update(pdp.getCurrent(entry.getKey()));
            }
        }
    }

    @Override
    public boolean isStale() {
        return (stalenessCount >= stalenessThreshold);
    }

    private void periodic() {
        if (lastBusVoltage == pdp.getVoltage()) stalenessCount++;
        else stalenessCount = 0;
    }

    public void setStalenessThreshold(int stalenessThreshold) {
        this.stalenessThreshold = stalenessThreshold;
    }

    public int getStalenessThreshold() {
        return stalenessThreshold;
    }

    @Override
    public PowerDistributionChannel getChannel(int channelID) {
        if (0 <= channelID && channelID < numChannels)
            return new PowerDistributionChannel(
                    channelID,
                    () -> {
                        return pdp.getCurrent(channelID);
                    });
        else return null;
    }
}
