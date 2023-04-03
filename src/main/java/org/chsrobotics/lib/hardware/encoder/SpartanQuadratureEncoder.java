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

import edu.wpi.first.wpilibj.Encoder;

public class SpartanQuadratureEncoder extends AbstractIncrementalEncoder {
    public static record QuadratureConfig(
            int channelA, int channelB, boolean inverted, double unitsPerCount) {
        public static QuadratureConfig getDefault() {
            return new QuadratureConfig(0, 1, false, 1);
        }

        public QuadratureConfig setChannels(int channelA, int channelB) {
            return new QuadratureConfig(channelA, channelB, inverted, unitsPerCount);
        }

        public QuadratureConfig setInverted(boolean inverted) {
            return new QuadratureConfig(channelA, channelB, inverted, unitsPerCount);
        }

        public QuadratureConfig setUnitsPerCount(int unitsPerCount) {
            return new QuadratureConfig(channelA, channelB, inverted, unitsPerCount);
        }
    }

    private final QuadratureConfig config;
    private final Encoder encoder;

    public SpartanQuadratureEncoder(QuadratureConfig config) {
        this.config = config;

        this.encoder = new Encoder(config.channelA, config.channelB);
    }

    public QuadratureConfig getConfig() {
        return config;
    }

    @Override
    public double getUnitsPerCount() {
        return config.unitsPerCount;
    }

    @Override
    public boolean getInverted() {
        return config.inverted;
    }

    @Override
    public double getRawCounts() {
        return encoder.get();
    }
}
