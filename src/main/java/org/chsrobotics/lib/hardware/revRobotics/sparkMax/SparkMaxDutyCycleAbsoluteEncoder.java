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
package org.chsrobotics.lib.hardware.revRobotics.sparkMax;

import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.hardware.base.encoder.AbstractAbsoluteEncoder;
import org.chsrobotics.lib.hardware.revRobotics.sparkMax.SpartanSparkMAX.SparkMaxRemoteFeedbackDevice;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

// TODO docs
public class SparkMaxDutyCycleAbsoluteEncoder extends AbstractAbsoluteEncoder
        implements SparkMaxRemoteFeedbackDevice {
    public static record SparkMaxDutyCycleAbsoluteEncoderConfig(boolean inverted, double offset) {}

    private final SparkMaxAbsoluteEncoder encoder;

    private final SparkMaxDutyCycleAbsoluteEncoderConfig config;

    private int stalenessCount = 0;
    private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;

    protected SparkMaxDutyCycleAbsoluteEncoder(
            SparkMaxAbsoluteEncoder encoder, SparkMaxDutyCycleAbsoluteEncoderConfig config) {
        this.encoder = encoder;

        this.config = config;

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public SparkMaxDutyCycleAbsoluteEncoderConfig getConfig() {
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
        return encoder.getPosition() * (config.inverted ? -1 : 1);
    }

    // we don't override the getUnconvertedVelocity method because sparkmax natively handles
    // wrap,
    // which is counter the spec of getUnconvertedVelocity()

    @Override
    public double getConvertedVelocity() {
        // sparkmax native differentiation is better than ours
        return encoder.getVelocity() * (config.inverted ? -1 : 1);
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

    @Override
    public MotorFeedbackSensor getRevSensor() {
        return encoder;
    }
}
