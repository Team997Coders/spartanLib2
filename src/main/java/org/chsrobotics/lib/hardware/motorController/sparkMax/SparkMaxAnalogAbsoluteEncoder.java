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
package org.chsrobotics.lib.hardware.motorController.sparkMax;

import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxAnalogSensor;
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.hardware.encoder.AbstractAbsoluteEncoder;
import org.chsrobotics.lib.hardware.motorController.sparkMax.SpartanSparkMAX.SparkMaxRemoteFeedbackDevice;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

/** Wrapper class for an analog absolute encoder plugged into a SparkMax. */
public class SparkMaxAnalogAbsoluteEncoder extends AbstractAbsoluteEncoder
        implements SparkMaxRemoteFeedbackDevice {

    /**
     * Configuration for a SparkMaxAnalogAbsoluteEncoder.
     *
     * @param inverted Whether this encoder's output should be "flipped".
     * @param refVoltage Maximum voltage this sensor can return. Most likely 5v.
     * @param offset Offset to add to this encoder's value such that it is equal to 0 where it is
     *     expected to.
     */
    public static record SparkMaxAnalogAbsoluteEncoderConfig(
            boolean inverted, double refVoltage, double offset) {}

    private final SparkMaxAnalogSensor encoder;

    private final SparkMaxAnalogAbsoluteEncoderConfig config;

    private int stalenessCount = 0;
    private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;

    protected SparkMaxAnalogAbsoluteEncoder(
            SparkMaxAnalogSensor encoder, SparkMaxAnalogAbsoluteEncoderConfig config) {
        this.encoder = encoder;

        this.config = config;

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    /**
     * Returns the configuration used by this encoder.
     *
     * @return This encoder's configuration.
     */
    public SparkMaxAnalogAbsoluteEncoderConfig getConfig() {
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
        return (encoder.getVoltage() / config.refVoltage) * (config.inverted ? -1 : 1);
    }

    @Override
    public boolean isStale() {
        return (stalenessCount >= stalenessThreshold);
    }

    /** Resets currently-running staleness counter. */
    public void resetStalenessCount() {
        stalenessCount = 0;
    }

    /**
     * Sets the threshold of unchanged cycles for {@code isStale()} to return true.
     *
     * @param cycles Number of cycles sensor voltage must stay the same for this to be considered
     *     potentially stale.
     */
    public void setStalenessThreshold(int cycles) {
        stalenessThreshold = cycles;
    }

    private void periodic() {
        if (getRawVelocity() == 0) stalenessCount++;
        else resetStalenessCount();
    }

    @Override
    public MotorFeedbackSensor getRevSensor() {
        return encoder;
    }
}
