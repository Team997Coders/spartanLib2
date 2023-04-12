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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.chsrobotics.lib.math.UtilityMath;

public class SpartanDutyCycleAbsoluteEncoder extends AbstractAbsoluteEncoder {
    public static record DutyCycleAbsoluteEncoderConfig(
            int channel, boolean inverted, double offset) {
        public DutyCycleAbsoluteEncoderConfig setChannel(int channel) {
            return new DutyCycleAbsoluteEncoderConfig(channel, inverted, offset);
        }

        public DutyCycleAbsoluteEncoderConfig setInverted(boolean inverted) {
            return new DutyCycleAbsoluteEncoderConfig(channel, inverted, offset);
        }

        public DutyCycleAbsoluteEncoderConfig setOffset(double offset) {
            return new DutyCycleAbsoluteEncoderConfig(channel, inverted, offset);
        }
    }

    private final DutyCycleEncoder encoder;

    private final DutyCycleAbsoluteEncoderConfig config;

    public SpartanDutyCycleAbsoluteEncoder(DutyCycleAbsoluteEncoderConfig config) {
        this.config = config;

        encoder = new DutyCycleEncoder(config.channel);
    }

    public DutyCycleAbsoluteEncoderConfig getConfig() {
        return config;
    }

    @Override
    public double getOffset() {
        return config.offset;
    }

    @Override
    public double getUnoffsetConvertedPosition() {
        return UtilityMath.normalizeAngleRadians(getRawPosition() * 2 * Math.PI);
    }

    @Override
    public double getRawPosition() {
        if (config.inverted) return -encoder.getAbsolutePosition();
        else return encoder.getAbsolutePosition();
    }

    @Override
    public boolean isStale() {
        return !encoder.isConnected();
    }
}
