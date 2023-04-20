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
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public class SpartanQuadratureEncoder extends AbstractEncoder {
    public static record QuadratureConfig(
            int channelA, int channelB, boolean inverted, double countsPerRotation) {

        public QuadratureConfig setChannels(int channelA, int channelB) {
            return new QuadratureConfig(channelA, channelB, inverted, countsPerRotation);
        }

        public QuadratureConfig setInverted(boolean inverted) {
            return new QuadratureConfig(channelA, channelB, inverted, countsPerRotation);
        }

        public QuadratureConfig setCountsPerRotation(int countsPerRotation) {
            return new QuadratureConfig(channelA, channelB, inverted, countsPerRotation);
        }
    }

    private final QuadratureConfig config;
    private final Encoder encoder;

    private int stalenessCount = 0;
    private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;

    public SpartanQuadratureEncoder(QuadratureConfig config) {
        this.config = config;

        this.encoder = new Encoder(config.channelA, config.channelB);

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public QuadratureConfig getConfig() {
        return config;
    }

    @Override
    public double getRawPosition() {
        if (config.inverted) return -encoder.get();
        else return encoder.get();
    }

    @Override
    public double getConvertedPosition() {
        return unitConversion(getRawPosition());
    }

    @Override
    public double getRawVelocity() {
        // FPGA can do differentiation better than us
        if (config.inverted) return -encoder.getRate();
        else return encoder.getRate();
    }

    @Override
    public double getConvertedVelocity() {
        return unitConversion(getRawVelocity());
    }

    @Override
    public boolean isStale() {
        return (stalenessCount >= stalenessThreshold);
    }

    public void resetStalenessCount() {
        stalenessCount = 0;
    }

    public void setStalenessThreshold(int cycles) {
        stalenessThreshold = cycles;
    }

    private void periodic() {
        if (getRawVelocity() == 0) stalenessCount++;
        else resetStalenessCount();
    }

    // maps "ticks" to radians
    private double unitConversion(double in) {
        return in * ((Math.PI * 2) / config.countsPerRotation);
    }
}
