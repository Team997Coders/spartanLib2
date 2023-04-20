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
package org.chsrobotics.lib.hardware.misc;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.telemetry.IntrinsicLoggable;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;
import org.chsrobotics.lib.util.Tuple2;

/** Wrapper class around both the Power Distribution Hub and the Power Distribution Panel. */
public class SpartanPowerDistribution implements IntrinsicLoggable, StalenessWatchable {

    public static enum HardwareType {
        PDH,
        PDP,
        NONE
    }

    /** Object representing a single non-switchable channel on either the PDH or PDP. */
    public class PowerDistributionChannel {
        private final int channel;

        private final DoubleSupplier currentSupplier;

        private final BooleanSupplier breakerFaultSupplier;

        private final BooleanSupplier stickyBreakerFaultSupplier;

        private final boolean isActualChannel;

        private final HardwareType type;

        private PowerDistributionChannel(
                int channel,
                DoubleSupplier currentSupplier,
                BooleanSupplier breakerFault,
                BooleanSupplier stickyBreakerFault,
                HardwareType type) {
            this.channel = channel;

            this.currentSupplier = currentSupplier;

            this.breakerFaultSupplier = breakerFault;

            this.stickyBreakerFaultSupplier = stickyBreakerFault;

            isActualChannel = true;

            this.type = type;
        }

        private PowerDistributionChannel() {
            this.channel = -1;

            this.currentSupplier = () -> -1;

            this.breakerFaultSupplier = () -> false;

            this.stickyBreakerFaultSupplier = () -> false;

            isActualChannel = false;

            type = HardwareType.NONE;
        }

        /**
         * Returns whether this object is an actual representation of a channel, or just the result
         * of some invalid function call.
         *
         * @return Whether this should be considered an actual, usable channel.
         */
        public boolean isActualChannel() {
            return isActualChannel;
        }

        /**
         * Returns the hardware ID of this channel.
         *
         * @return The hardware index of this channel, as seen on the enclosure of the boards.
         */
        public int getChannel() {
            return channel;
        }

        /**
         * Returns the present measured current, in amps, through this channel.
         *
         * @return The present current.
         */
        public double getCurrentAmps() {
            return currentSupplier.getAsDouble();
        }

        /**
         * Returns whether there is presently a breaker fault (popped/not present/dislodged) on this
         * channel.
         *
         * <p>Only supported on PDH hardware! If this is invoked on PDP hardware, will return {@code
         * false}.
         *
         * @return Whether this channel reports a breaker fault.
         */
        public boolean getBreakerFault() {
            return breakerFaultSupplier.getAsBoolean();
        }

        /**
         * Returns whether this channel has had a breaker fault (popped/not present/dislodged) since
         * this device reset.
         *
         * <p>Only supported on PDH hardware! If this is invoked on PDP hardware, will return {@code
         * false}.
         *
         * @return Whether this channel has had a breaker fault.
         */
        public boolean getBreakerStickyFault() {
            return stickyBreakerFaultSupplier.getAsBoolean();
        }

        /**
         * Returns the hardware type of the module this channel belongs to.
         *
         * @return The type of this channel's parent.
         */
        public HardwareType getParentHardwareType() {
            return type;
        }
    }

    /** Object representing a single switchable channel on the PDH. */
    public class PowerDistributionSwitchableChannel extends PowerDistributionChannel {
        private boolean state;

        private final BooleanConsumer setStateMethod;

        private PowerDistributionSwitchableChannel(
                DoubleSupplier currentSupplier,
                BooleanSupplier breakerFaultSupplier,
                BooleanSupplier stickyBreakerFaultSupplier,
                boolean initialState,
                BooleanConsumer setStateMethod) {
            super(
                    23,
                    currentSupplier,
                    breakerFaultSupplier,
                    stickyBreakerFaultSupplier,
                    HardwareType.PDH);

            this.state = initialState;

            this.setStateMethod = setStateMethod;
        }

        private PowerDistributionSwitchableChannel() {
            super();

            this.state = false;

            setStateMethod = (b) -> {};
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

        /**
         * Sets the state of this switchable channel (true = on).
         *
         * <p>Non-op if this is not an actual channel.
         *
         * @param state Whether this channel should be turned on.
         */
        public void setSwitchableState(boolean state) {
            if (isActualChannel() && this.state != state) {
                setStateMethod.accept(state);

                this.state = state;
            }
        }
    }

    /**
     * Configuration for a SpartanPowerDistribution object.
     *
     * @param canID The canbus identifier of the module. Default is 0 for PDP, 1 for PDH.
     * @param type What kind of hardware this represents.
     * @param initialSwitchableState The state to initially set the switchable channel to (true =
     *     on).
     */
    public static record PowerDistributionConfig(
            int canID, HardwareType type, boolean initialSwitchableState) {
        public static PowerDistributionConfig getDefaultPDH() {
            return new PowerDistributionConfig(1, HardwareType.PDH, false);
        }

        public static PowerDistributionConfig getDefaultPDP() {
            return new PowerDistributionConfig(0, HardwareType.PDP, false);
        }

        public PowerDistributionConfig setCanID(int canID) {
            return new PowerDistributionConfig(canID, type, initialSwitchableState);
        }

        public PowerDistributionConfig setType(HardwareType type) {
            return new PowerDistributionConfig(canID, type, initialSwitchableState);
        }

        public PowerDistributionConfig setInitialSwitchableState(boolean initialSwitchableState) {
            return new PowerDistributionConfig(canID, type, initialSwitchableState);
        }
    }

    private static record ChannelLoggersPackage(
            Logger<Double> currentLogger,
            Logger<Boolean> breakerFaultLogger,
            Logger<Boolean> breakerStickyFaultLogger) {}

    private final int numPDHChannels = 23;
    private final int numPDPChannels = 15;

    private final PowerDistribution powerDistribution;

    private final PowerDistributionConfig config;

    private final HashMap<Integer, ChannelLoggersPackage> channelLoggersMap = new HashMap<>();

    private Logger<String[]> otherFaultsLogger;
    private Logger<String[]> otherStickyFaultsLogger;

    private Logger<Double> busVoltageLogger;
    private Logger<Double> totalCurrentLogger;
    private Logger<Double> totalEnergyUseLogger;

    private Logger<Double> temperatureCLogger;

    private Logger<Boolean> switchableChannelStateLogger;

    private Logger<Boolean> stalenessLogger;

    private boolean logsConstructed = false;

    private int stalenessCount = 0;

    private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;

    private double lastBusVoltage = 0;

    /**
     * Constructs a SpartanPowerDistribution.
     *
     * @param config Configuration to use for this.
     */
    public SpartanPowerDistribution(PowerDistributionConfig config) {
        this.config = config;

        this.powerDistribution =
                new PowerDistribution(
                        config.canID,
                        (config.type == HardwareType.PDH) ? ModuleType.kRev : ModuleType.kCTRE);

        configure();

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    /**
     * Returns the config used by this.
     *
     * @return The PowerDistributionConfig used by this object.
     */
    public PowerDistributionConfig getConfig() {
        return config;
    }

    /**
     * Gets a non-switchable PowerDistributionChannel object by its channel index.
     *
     * <p>If the index is out of range of what the hardware type allows ([0-15] for PDP, [0-22] for
     * PDH), returns a dummy PowerDistributionChannel with no significance.
     *
     * @param channelID Hardware index of the channel. Can be found printed on the plastic
     *     enclosure.
     * @return A PowerDistributionChannel representing the given channel.
     */
    public PowerDistributionChannel getChannel(int channelID) {
        if (config.type == HardwareType.PDP) {
            if (channelID >= 0 && channelID <= numPDPChannels) {
                return new PowerDistributionChannel(
                        channelID,
                        () -> powerDistribution.getCurrent(channelID),
                        () -> false,
                        () -> false,
                        config.type);
            } else return new PowerDistributionChannel(); // null channel
        } else if (channelID >= 0
                && channelID
                        < numPDHChannels) { // intentionally excluding last (switchable) channel
            return new PowerDistributionChannel(
                    channelID,
                    () -> powerDistribution.getCurrent(channelID),
                    () -> getBreakerFaultAtChannel(channelID).firstValue(),
                    () -> getBreakerFaultAtChannel(channelID).secondValue(),
                    config.type);
        } else return new PowerDistributionChannel(); // null channel
    }

    /**
     * Gets the PowerDistributionSwitchableChannel representing channel 23 of the PDH.
     *
     * <p>If invoked on returns a dummy PowerDistributionChannel with no significance.
     *
     * @return A PowerDistributionChannel representing the given channel.
     */
    public PowerDistributionSwitchableChannel getSwitchableChannel() {
        if (config.type == HardwareType.PDP)
            return new PowerDistributionSwitchableChannel(); // null channel
        else
            return new PowerDistributionSwitchableChannel(
                    () -> powerDistribution.getCurrent(numPDHChannels),
                    () -> getBreakerFaultAtChannel(numPDHChannels).firstValue(),
                    () -> getBreakerFaultAtChannel(numPDHChannels).secondValue(),
                    config.initialSwitchableState,
                    powerDistribution::setSwitchableChannel);
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {

        if (!logsConstructed) {
            LoggerFactory<Boolean> boolFactory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            LoggerFactory<Double> doubleFactory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            for (int i = 0;
                    i <= (config.type == HardwareType.PDH ? numPDPChannels : numPDHChannels);
                    i++) {
                String header = name + "/channel" + String.valueOf(i);

                channelLoggersMap.put(
                        i,
                        new ChannelLoggersPackage(
                                doubleFactory.getLogger(header + "/current_amps"),
                                boolFactory.getLogger(header + "/breakerFaultActive"),
                                boolFactory.getLogger(header + "/breakerStickyFaultActive")));
            }

            busVoltageLogger = doubleFactory.getLogger(name + "/busVoltage_v");
            totalCurrentLogger = doubleFactory.getLogger(name + "/totalCurrent_amps");
            totalEnergyUseLogger = doubleFactory.getLogger(name + "/totalEnergy_joules");

            temperatureCLogger = doubleFactory.getLogger(name + "/temperature_C");

            switchableChannelStateLogger = boolFactory.getLogger(name + "switchableChannelState");

            stalenessLogger = boolFactory.getLogger(name + "isStale");

            otherFaultsLogger =
                    new Logger<>(
                            log, name + "/generalFaults", subdirName, publishToNT, recordInLog);

            otherStickyFaultsLogger =
                    new Logger<>(
                            log,
                            name + "/generalStickyFaults",
                            subdirName,
                            publishToNT,
                            recordInLog);

            new Logger<>(log, name + "hardwareType", subdirName, publishToNT, recordInLog)
                    .update(config.type);

            PeriodicCallbackHandler.registerCallback(this::updateLogs);

            logsConstructed = true;
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            ArrayList<String> otherFaults = new ArrayList<>();

            var faults = powerDistribution.getFaults();
            if (faults.Brownout) otherFaults.add("brownout");
            if (faults.CanWarning) otherFaults.add("canWarning");
            if (faults.HardwareFault) otherFaults.add("hardwareFault");

            otherFaultsLogger.update(otherFaults.toArray(new String[] {}));

            ArrayList<String> otherStickyFaults = new ArrayList<>();

            var stickyFaults = powerDistribution.getStickyFaults();
            if (stickyFaults.Brownout) otherStickyFaults.add("brownout");
            if (stickyFaults.CanBusOff) otherStickyFaults.add("canBusOff");
            if (stickyFaults.CanWarning) otherStickyFaults.add("canWarning");
            if (stickyFaults.HasReset) otherStickyFaults.add("hasReset");

            otherStickyFaultsLogger.update(otherStickyFaults.toArray(new String[] {}));

            busVoltageLogger.update(powerDistribution.getVoltage());
            totalCurrentLogger.update(powerDistribution.getTotalCurrent());
            totalEnergyUseLogger.update(powerDistribution.getTotalEnergy());

            temperatureCLogger.update(powerDistribution.getTemperature());

            switchableChannelStateLogger.update(powerDistribution.getSwitchableChannel());

            stalenessLogger.update(isStale());

            for (var entry : channelLoggersMap.entrySet()) {
                entry.getValue().currentLogger.update(powerDistribution.getCurrent(entry.getKey()));

                entry.getValue()
                        .breakerFaultLogger
                        .update(getBreakerFaultAtChannel(entry.getKey()).firstValue());

                entry.getValue()
                        .breakerStickyFaultLogger
                        .update(getBreakerFaultAtChannel(entry.getKey()).secondValue());
            }
        }
    }

    // [fault, sticky fault]
    private Tuple2<Boolean> getBreakerFaultAtChannel(int channel) {
        // wpilib is the best library ever written
        switch (channel) {
            case 0:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel0BreakerFault,
                        powerDistribution.getStickyFaults().Channel0BreakerFault);
            case 1:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel1BreakerFault,
                        powerDistribution.getStickyFaults().Channel1BreakerFault);
            case 2:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel2BreakerFault,
                        powerDistribution.getStickyFaults().Channel2BreakerFault);
            case 3:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel3BreakerFault,
                        powerDistribution.getStickyFaults().Channel3BreakerFault);
            case 4:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel4BreakerFault,
                        powerDistribution.getStickyFaults().Channel4BreakerFault);
            case 5:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel5BreakerFault,
                        powerDistribution.getStickyFaults().Channel5BreakerFault);
            case 6:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel6BreakerFault,
                        powerDistribution.getStickyFaults().Channel6BreakerFault);
            case 7:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel7BreakerFault,
                        powerDistribution.getStickyFaults().Channel7BreakerFault);
            case 8:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel8BreakerFault,
                        powerDistribution.getStickyFaults().Channel8BreakerFault);
            case 9:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel9BreakerFault,
                        powerDistribution.getStickyFaults().Channel9BreakerFault);
            case 10:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel10BreakerFault,
                        powerDistribution.getStickyFaults().Channel10BreakerFault);
            case 11:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel11BreakerFault,
                        powerDistribution.getStickyFaults().Channel11BreakerFault);
            case 12:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel12BreakerFault,
                        powerDistribution.getStickyFaults().Channel12BreakerFault);
            case 13:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel13BreakerFault,
                        powerDistribution.getStickyFaults().Channel13BreakerFault);
            case 14:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel14BreakerFault,
                        powerDistribution.getStickyFaults().Channel14BreakerFault);
            case 15:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel15BreakerFault,
                        powerDistribution.getStickyFaults().Channel15BreakerFault);
            case 16:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel16BreakerFault,
                        powerDistribution.getStickyFaults().Channel16BreakerFault);
            case 17:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel17BreakerFault,
                        powerDistribution.getStickyFaults().Channel17BreakerFault);
            case 18:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel18BreakerFault,
                        powerDistribution.getStickyFaults().Channel18BreakerFault);
            case 19:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel19BreakerFault,
                        powerDistribution.getStickyFaults().Channel19BreakerFault);
            case 20:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel20BreakerFault,
                        powerDistribution.getStickyFaults().Channel20BreakerFault);
            case 21:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel21BreakerFault,
                        powerDistribution.getStickyFaults().Channel21BreakerFault);
            case 22:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel22BreakerFault,
                        powerDistribution.getStickyFaults().Channel22BreakerFault);
            case 23:
                return Tuple2.of(
                        powerDistribution.getFaults().Channel23BreakerFault,
                        powerDistribution.getStickyFaults().Channel23BreakerFault);
            default:
                return Tuple2.of(false, false);
        }
    }

    private void configure() {
        powerDistribution.clearStickyFaults();

        powerDistribution.setSwitchableChannel(config.initialSwitchableState);
    }

    private void periodic() {
        if (powerDistribution.getStickyFaults().HasReset) configure();

        if (powerDistribution.getVoltage() == lastBusVoltage) stalenessCount++;
        else resetStalenessCount();

        lastBusVoltage = powerDistribution.getVoltage();
    }

    @Override
    /** {@inheritDoc} */
    public boolean isStale() {
        return (stalenessCount >= stalenessThreshold);
    }

    /**
     * Resets the number of accumulated counts used to determine when this can be considered stale.
     */
    public void resetStalenessCount() {
        stalenessCount = 0;
    }

    /**
     * Sets the number of staleness counts required to be considered stale.
     *
     * @param cycles The new threshold, in code cycles.
     */
    public void setStalenessThreshold(int cycles) {
        stalenessThreshold = cycles;
    }
}
