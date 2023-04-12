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

import edu.wpi.first.wpilibj.AnalogEncoder;
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public class SpartanAnalogAbsoluteEncoder extends AbstractAbsoluteEncoder {
    public static record AnalogAbsoluteEncoderConfig(int channel, boolean inverted, double offset) {
        public AnalogAbsoluteEncoderConfig setChannel(int channel) {
            return new AnalogAbsoluteEncoderConfig(channel, inverted, offset);
        }

        public AnalogAbsoluteEncoderConfig setInverted(boolean inverted) {
            return new AnalogAbsoluteEncoderConfig(channel, inverted, offset);
        }

        public AnalogAbsoluteEncoderConfig setOffset(double offset) {
            return new AnalogAbsoluteEncoderConfig(channel, inverted, offset);
        }
    }

    private final AnalogAbsoluteEncoderConfig config;

    private final AnalogEncoder encoder;

    private int stalenessCount = 0;
    private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;

    public SpartanAnalogAbsoluteEncoder(AnalogAbsoluteEncoderConfig config) {
        this.config = config;

        this.encoder = new AnalogEncoder(config.channel);

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public AnalogAbsoluteEncoderConfig getConfig() {
        return config;
    }

    @Override
    public double getOffset() {
        return config.offset;
    }

    @Override
    public double getUnoffsetConvertedPosition() {
        return UtilityMath.normalizeAngleRadians(2 * Math.PI * getRawPosition());
    }

    @Override
    public double getRawPosition() {
        if (config.inverted) return -encoder.getAbsolutePosition();
        else return encoder.getAbsolutePosition();
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
}
