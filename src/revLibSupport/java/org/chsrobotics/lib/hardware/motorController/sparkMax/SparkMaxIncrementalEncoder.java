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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.hardware.encoder.AbstractEncoder;
import org.chsrobotics.lib.hardware.motorController.sparkMax.SpartanSparkMAX.SparkMaxRemoteFeedbackDevice;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

// TODO docs
public class SparkMaxIncrementalEncoder extends AbstractEncoder
        implements SparkMaxRemoteFeedbackDevice {
    public static record SparkMaxIncrementalEncoderConfig(
            Type type, boolean inverted, double countsPerRevolution) {}

    private final SparkMaxIncrementalEncoderConfig config;

    private final RelativeEncoder encoder;

    private int stalenessCount = 0;
    private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;

    protected SparkMaxIncrementalEncoder(
            SparkMaxIncrementalEncoderConfig config, RelativeEncoder encoder) {
        this.config = config;

        this.encoder = encoder;

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public SparkMaxIncrementalEncoderConfig getConfig() {
        return config;
    }

    @Override
    public double getRawPosition() {
        return encoder.getPosition() * (config.inverted ? -1 : 1);
    }

    @Override
    public double getConvertedPosition() {
        return unitConversion(getRawPosition());
    }

    @Override
    public double getRawVelocity() {
        // native sparkmax differentiation is better than ours
        return encoder.getVelocity() * (config.inverted ? -1 : 1) / 60;
        // convert from RPM to RPS, for consistency
    }

    @Override
    public double getConvertedVelocity() {
        return unitConversion(getRawVelocity());
    }

    private double unitConversion(double in) {
        return in * 2 * Math.PI / config.countsPerRevolution;
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
