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
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;
import org.chsrobotics.lib.util.Tuple2;

// TODO: finish docs
public class SpartanPDH extends AbstractPowerDistribution {
    public class PDHChannel extends PowerDistributionChannel {

        private final BooleanSupplier faultSupplier;
        private final BooleanSupplier stickyFaultSupplier;

        private PDHChannel(
                int channel,
                DoubleSupplier currentSupplier,
                BooleanSupplier faultSupplier,
                BooleanSupplier stickyFaultSupplier) {
            super(channel, currentSupplier);

            this.faultSupplier = faultSupplier;
            this.stickyFaultSupplier = stickyFaultSupplier;
        }

        /**
         * Returns whether there is presently a breaker fault (popped/not present/dislodged) on this
         * channel.
         *
         * @return Whether this channel reports a breaker fault.
         */
        public boolean getBreakerFault() {
            return faultSupplier.getAsBoolean();
        }

        /**
         * Returns whether this channel has had a breaker fault (popped/not present/dislodged) since
         * this device reset.
         *
         * @return Whether this channel has had a breaker fault.
         */
        public boolean getBreakerStickyFault() {
            return stickyFaultSupplier.getAsBoolean();
        }
    }

    public class SwitchablePDHChannel extends PDHChannel {

        private final BooleanConsumer setStateMethod;

        private boolean state;

        private SwitchablePDHChannel(
                int channel,
                DoubleSupplier currentSupplier,
                BooleanSupplier faultSupplier,
                BooleanSupplier stickyFaultSupplier,
                boolean initialState,
                BooleanConsumer setStateMethod) {
            super(channel, currentSupplier, faultSupplier, stickyFaultSupplier);

            this.setStateMethod = setStateMethod;

            this.state = initialState;

            setSwitchableState(state);
        }

        /**
         * Sets the state of this switchable channel (true = on).
         *
         * <p>Non-op if this is not an actual channel.
         *
         * @param state Whether this channel should be turned on.
         */
        public void setSwitchableState(boolean state) {
            if (this.state != state) {
                setStateMethod.accept(state);

                this.state = state;
            }
        }

        /**
         * Returns whether the current state of this switchable channel (true = on).
         *
         * <p>If this is not an actual channel, will return {@code false}.
         *
         * @return Whether the channel is turned on.
         */
        public boolean getSwitchableState() {
            return state;
        }
    }

    public static record PDHConfig(int canID) {
        public static PDHConfig getAutoCanID() {
            return new PDHConfig(1);
        }
    }

    private record ChannelLoggersPackage(
            Logger<Double> currentLogger,
            Logger<Boolean> faultLogger,
            Logger<Boolean> stickyFaultLogger) {

        // header should exclude trailing "/"
        private static ChannelLoggersPackage makeDefault(
                String header,
                LoggerFactory<Boolean> boolFactory,
                LoggerFactory<Double> doubleFactory) {
            return new ChannelLoggersPackage(
                    doubleFactory.getLogger(header + "/current_A"),
                    boolFactory.getLogger(header + "/breakerFaultActive"),
                    boolFactory.getLogger(header + "/breakerStickyFaultActive"));
        }
    }

    // indexing of channels is 0-based-- 22 is actual highest channel num (excluding switchable)
    private final int numChannels = 23;

    private final int switchableChannelIndex = 23;

    private final PDHConfig config;

    private final PowerDistribution pdh;

    private final Map<Integer, ChannelLoggersPackage> channelLoggersMap = new HashMap<>();

    private Logger<Boolean> switchableStateLogger;

    private Logger<Double> temperatureCLogger;

    private Logger<Double> totalCurrentALogger;
    private Logger<Double> accumulatedEnergyJLogger;

    private Logger<Double> busVoltageVLogger;

    private boolean logsConstructed = false;

    private int stalenessCount = 0;
    private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;
    private double lastBusVoltage = 0;

    public SpartanPDH(PDHConfig config) {
        this.config = config;

        pdh = new PowerDistribution(defaultStalenessThresholdCycles, ModuleType.kRev);

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public PDHConfig getConfig() {
        return config;
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            LoggerFactory<Boolean> boolFactory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            LoggerFactory<Double> doubleFactory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            for (int i = 0; i < numChannels; i++) {
                channelLoggersMap.put(
                        i,
                        ChannelLoggersPackage.makeDefault(
                                name + "/channel" + String.valueOf(i), boolFactory, doubleFactory));
            }

            channelLoggersMap.put(
                    switchableChannelIndex,
                    ChannelLoggersPackage.makeDefault(
                            name + "/switchableChannel", boolFactory, doubleFactory));

            switchableStateLogger = boolFactory.getLogger("/switchableChannel/active");

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
            for (var entry : channelLoggersMap.entrySet()) {
                int channel = entry.getKey();

                entry.getValue().currentLogger.update(pdh.getCurrent(channel));

                entry.getValue().faultLogger.update(getBreakerFaultAtChannel(channel).firstValue());

                entry.getValue()
                        .stickyFaultLogger
                        .update(getBreakerFaultAtChannel(channel).secondValue());
            }

            switchableStateLogger.update(pdh.getSwitchableChannel());

            temperatureCLogger.update(pdh.getTemperature());

            totalCurrentALogger.update(pdh.getTotalCurrent());
            accumulatedEnergyJLogger.update(pdh.getTotalEnergy());

            busVoltageVLogger.update(pdh.getVoltage());
        }
    }

    @Override
    public boolean isStale() {
        return (stalenessCount >= stalenessThreshold);
    }

    private void periodic() {
        if (lastBusVoltage == pdh.getVoltage()) stalenessCount++;
        else stalenessCount = 0;
    }

    public void setStalenessThreshold(int stalenessThreshold) {
        this.stalenessThreshold = stalenessThreshold;
    }

    public int getStalenessThreshold() {
        return stalenessThreshold;
    }

    @Override
    public PDHChannel getChannel(int channelID) {
        if (0 <= channelID && channelID <= numChannels)
            return new PDHChannel(
                    channelID,
                    () -> {
                        return pdh.getCurrent(channelID);
                    },
                    () -> {
                        return getBreakerFaultAtChannel(channelID).firstValue();
                    },
                    () -> {
                        return getBreakerFaultAtChannel(channelID).secondValue();
                    });
        else return null;
    }

    public SwitchablePDHChannel getSwitchableChannel(boolean initialState) {
        return new SwitchablePDHChannel(
                switchableChannelIndex,
                () -> {
                    return pdh.getCurrent(switchableChannelIndex);
                },
                () -> {
                    return getBreakerFaultAtChannel(switchableChannelIndex).firstValue();
                },
                () -> {
                    return getBreakerFaultAtChannel(switchableChannelIndex).secondValue();
                },
                initialState,
                pdh::setSwitchableChannel);
    }

    // [fault, sticky fault]
    private Tuple2<Boolean> getBreakerFaultAtChannel(int channel) {
        // wpilib is the best library ever written

        var faults = pdh.getFaults();
        var stickyFaults = pdh.getStickyFaults();

        switch (channel) {
            case 0:
                return Tuple2.of(faults.Channel0BreakerFault, stickyFaults.Channel0BreakerFault);
            case 1:
                return Tuple2.of(faults.Channel1BreakerFault, stickyFaults.Channel1BreakerFault);
            case 2:
                return Tuple2.of(faults.Channel2BreakerFault, stickyFaults.Channel2BreakerFault);
            case 3:
                return Tuple2.of(faults.Channel3BreakerFault, stickyFaults.Channel3BreakerFault);
            case 4:
                return Tuple2.of(faults.Channel4BreakerFault, stickyFaults.Channel4BreakerFault);
            case 5:
                return Tuple2.of(faults.Channel5BreakerFault, stickyFaults.Channel5BreakerFault);
            case 6:
                return Tuple2.of(faults.Channel6BreakerFault, stickyFaults.Channel6BreakerFault);
            case 7:
                return Tuple2.of(faults.Channel7BreakerFault, stickyFaults.Channel7BreakerFault);
            case 8:
                return Tuple2.of(faults.Channel8BreakerFault, stickyFaults.Channel8BreakerFault);
            case 9:
                return Tuple2.of(faults.Channel9BreakerFault, stickyFaults.Channel9BreakerFault);
            case 10:
                return Tuple2.of(faults.Channel10BreakerFault, stickyFaults.Channel10BreakerFault);
            case 11:
                return Tuple2.of(faults.Channel11BreakerFault, stickyFaults.Channel11BreakerFault);
            case 12:
                return Tuple2.of(faults.Channel12BreakerFault, stickyFaults.Channel12BreakerFault);
            case 13:
                return Tuple2.of(faults.Channel13BreakerFault, stickyFaults.Channel13BreakerFault);
            case 14:
                return Tuple2.of(faults.Channel14BreakerFault, stickyFaults.Channel14BreakerFault);
            case 15:
                return Tuple2.of(faults.Channel15BreakerFault, stickyFaults.Channel15BreakerFault);
            case 16:
                return Tuple2.of(faults.Channel16BreakerFault, stickyFaults.Channel16BreakerFault);
            case 17:
                return Tuple2.of(faults.Channel17BreakerFault, stickyFaults.Channel17BreakerFault);
            case 18:
                return Tuple2.of(faults.Channel18BreakerFault, stickyFaults.Channel18BreakerFault);
            case 19:
                return Tuple2.of(faults.Channel19BreakerFault, stickyFaults.Channel19BreakerFault);
            case 20:
                return Tuple2.of(faults.Channel20BreakerFault, stickyFaults.Channel20BreakerFault);
            case 21:
                return Tuple2.of(faults.Channel21BreakerFault, stickyFaults.Channel21BreakerFault);
            case 22:
                return Tuple2.of(faults.Channel22BreakerFault, stickyFaults.Channel22BreakerFault);
            case 23:
                return Tuple2.of(faults.Channel23BreakerFault, stickyFaults.Channel23BreakerFault);
            default:
                return Tuple2.of(false, false);
        }
    }
}
