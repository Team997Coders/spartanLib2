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
package org.chsrobotics.lib.hardware.powerDistribution;

import java.util.function.DoubleSupplier;
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.telemetry.IntrinsicLoggable;

// TODO: finish docs
public abstract class AbstractPowerDistribution implements IntrinsicLoggable, StalenessWatchable {
    /** Object representing a single non-switchable channel on either the PDH or PDP. */
    public class PowerDistributionChannel {
        private final int channel;

        private final DoubleSupplier currentSupplier;

        PowerDistributionChannel(int channel, DoubleSupplier currentSupplier) {
            this.channel = channel;

            this.currentSupplier = currentSupplier;
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
    }

    /**
     * Gets a non-switchable PowerDistributionChannel object by its channel index.
     *
     * @param channelID Hardware index of the channel. Can be found printed on the plastic
     *     enclosure.
     * @return A PowerDistributionChannel representing the given channel, or {@code null} if {@code
     *     channelID} is invalid.
     */
    public abstract PowerDistributionChannel getChannel(int channelID);
}
