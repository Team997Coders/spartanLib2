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
package org.chsrobotics.lib.hardware.imu;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.datalog.DataLog;
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.telemetry.IntrinsicLoggable;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public abstract class AbstractIMU implements IntrinsicLoggable, StalenessWatchable {
    private boolean logsConstructed = false;

    private Logger<Double> unoffsetAccelXLogger;
    private Logger<Double> unoffsetAccelYLogger;
    private Logger<Double> unoffsetAccelZLogger;

    private Logger<Double> offsetAccelXLogger;
    private Logger<Double> offsetAccelYLogger;
    private Logger<Double> offsetAccelZLogger;

    private Logger<Double> unoffsetPitchLogger;
    private Logger<Double> unoffsetYawLogger;
    private Logger<Double> unoffsetRollLogger;

    private Logger<Double> offsetPitchLogger;
    private Logger<Double> offsetYawLogger;
    private Logger<Double> offsetRollLogger;

    private Logger<Double> unoffsetPitchVLogger;
    private Logger<Double> unoffsetYawVLogger;
    private Logger<Double> unoffsetRollVLogger;

    private Logger<Double> offsetPitchVLogger;
    private Logger<Double> offsetYawVLogger;
    private Logger<Double> offsetRollVLogger;

    private Logger<Double> unoffsetPitchALogger;
    private Logger<Double> unoffsetYawALogger;
    private Logger<Double> unoffsetRollALogger;

    private Logger<Double> offsetPitchALogger;
    private Logger<Double> offsetYawALogger;
    private Logger<Double> offsetRollALogger;

    private final DifferentiatingFilter unoffsetPitchVFilter = new DifferentiatingFilter(true);
    private final DifferentiatingFilter unoffsetYawVFilter = new DifferentiatingFilter(true);
    private final DifferentiatingFilter unoffsetRollVFilter = new DifferentiatingFilter(true);

    private final DifferentiatingFilter offsetPitchVFilter = new DifferentiatingFilter(true);
    private final DifferentiatingFilter offsetYawVFilter = new DifferentiatingFilter(true);
    private final DifferentiatingFilter offsetRollVFilter = new DifferentiatingFilter(true);

    private final DifferentiatingFilter unoffsetPitchAFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter unoffsetYawAFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter unoffsetRollAFilter = new DifferentiatingFilter();

    private final DifferentiatingFilter offsetPitchAFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter offsetYawAFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter offsetRollAFilter = new DifferentiatingFilter();

    private Logger<Boolean> stalenessWatchdogTriggeredLogger;

    public AbstractIMU() {
        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            LoggerFactory<Double> factory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            unoffsetAccelXLogger = factory.getLogger(name + "/unoffsetAccelX_m_per_s_squared");
            unoffsetAccelYLogger = factory.getLogger(name + "/unoffsetAccelY_m_per_s_squared");
            unoffsetAccelZLogger = factory.getLogger(name + "/unoffsetAccelZ_m_per_s_squared");

            offsetAccelXLogger = factory.getLogger(name + "/offsetAccelX_m_per_s_squared");
            offsetAccelYLogger = factory.getLogger(name + "/offsetAccelY_m_per_s_squared");
            offsetAccelZLogger = factory.getLogger(name + "/offsetAccelZ_m_per_s_squared");

            unoffsetPitchLogger = factory.getLogger(name + "/unoffsetPitch_rad");
            unoffsetYawLogger = factory.getLogger(name + "/unoffsetYaw_rad");
            unoffsetRollLogger = factory.getLogger(name + "/unoffsetRoll_rad");

            offsetPitchLogger = factory.getLogger(name + "/offsetPitch_rad");
            offsetYawLogger = factory.getLogger(name + "/offsetYaw_rad");
            offsetRollLogger = factory.getLogger(name + "/offsetRoll_rad");

            unoffsetPitchVLogger = factory.getLogger(name + "/unoffsetPitchVelocity_rad_per_s");
            unoffsetYawVLogger = factory.getLogger(name + "/unoffsetYawVelocity_rad_per_s");
            unoffsetRollVLogger = factory.getLogger(name + "/unoffsetRollVelocity_rad_per_s");

            offsetPitchVLogger = factory.getLogger(name + "/offsetPitchVelocity_rad_per_s");
            offsetYawVLogger = factory.getLogger(name + "/offsetYawVelocity_rad_per_s");
            offsetRollVLogger = factory.getLogger(name + "/offsetRollVelocity_rad_per_s");

            unoffsetPitchALogger =
                    factory.getLogger(name + "/unoffsetPitchAcceleration_rad_per_s_squared");
            unoffsetYawALogger =
                    factory.getLogger(name + "/unoffsetYawAcceleration_rad_per_s_squared");
            unoffsetRollALogger =
                    factory.getLogger(name + "/unoffsetRollAcceleration_rad_per_s_squared");

            offsetPitchALogger =
                    factory.getLogger(name + "/offsetPitchAcceleration_rad_per_s_squared");
            offsetYawALogger = factory.getLogger(name + "/offsetYawAcceleration_rad_per_s_squared");
            offsetRollALogger =
                    factory.getLogger(name + "/offsetRollAcceleration_rad_per_s_squared");

            stalenessWatchdogTriggeredLogger =
                    new Logger<>(log, name + "/isStale", subdirName, publishToNT, recordInLog);

            logsConstructed = true;
            PeriodicCallbackHandler.registerCallback(this::updateLogs);
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            unoffsetAccelXLogger.update(getRawAccelerationX());
            unoffsetAccelYLogger.update(getRawAccelerationY());
            unoffsetAccelZLogger.update(getRawAccelerationZ());

            offsetAccelXLogger.update(getOffsetAccelerationX());
            offsetAccelYLogger.update(getOffsetAccelerationY());
            offsetAccelZLogger.update(getOffsetAccelerationZ());

            unoffsetPitchLogger.update(getRawPitch());
            unoffsetYawLogger.update(getRawYaw());
            unoffsetRollLogger.update(getRawRoll());

            offsetPitchLogger.update(getOffsetPitch());
            offsetYawLogger.update(getOffsetYaw());
            offsetRollLogger.update(getOffsetRoll());

            unoffsetPitchVLogger.update(getRawPitchVelocity());
            unoffsetYawVLogger.update(getRawYawVelocity());
            unoffsetRollVLogger.update(getRawRollVelocity());

            offsetPitchVLogger.update(getOffsetPitchVelocity());
            offsetYawVLogger.update(getOffsetYawVelocity());
            offsetRollVLogger.update(getOffsetRollVelocity());

            unoffsetPitchALogger.update(getRawPitchAcceleration());
            unoffsetYawALogger.update(getRawYawAcceleration());
            unoffsetRollALogger.update(getRawRollAcceleration());

            offsetPitchALogger.update(getOffsetPitchAcceleration());
            offsetYawALogger.update(getOffsetYawAcceleration());
            offsetRollALogger.update(getOffsetRollAcceleration());

            stalenessWatchdogTriggeredLogger.update(isStale());
        }
    }

    private void periodic(double dtSeconds) {
        unoffsetPitchVFilter.calculate(getRawOrientation().getY(), dtSeconds);
        unoffsetYawVFilter.calculate(getRawOrientation().getZ(), dtSeconds);
        unoffsetRollVFilter.calculate(getRawOrientation().getX(), dtSeconds);

        offsetPitchVFilter.calculate(getOffsetOrientation().getY(), dtSeconds);
        offsetYawVFilter.calculate(getOffsetOrientation().getZ(), dtSeconds);
        offsetRollVFilter.calculate(getOffsetOrientation().getX(), dtSeconds);

        unoffsetPitchAFilter.calculate(unoffsetPitchVFilter.getCurrentOutput(), dtSeconds);
        unoffsetYawAFilter.calculate(unoffsetYawVFilter.getCurrentOutput(), dtSeconds);
        unoffsetRollAFilter.calculate(unoffsetRollVFilter.getCurrentOutput(), dtSeconds);

        offsetPitchAFilter.calculate(offsetPitchVFilter.getCurrentOutput(), dtSeconds);
        offsetYawAFilter.calculate(offsetPitchVFilter.getCurrentOutput(), dtSeconds);
        offsetRollAFilter.calculate(offsetRollVFilter.getCurrentOutput(), dtSeconds);
    }

    public abstract Rotation3d getRotationOffset();

    public Translation3d getOffsetAccelerationVector() {
        return getRawAccelerationVector().rotateBy(getRotationOffset());
    }

    public abstract Translation3d getRawAccelerationVector();

    public abstract Rotation3d getRawOrientation();

    public Rotation3d getOffsetOrientation() {
        return getRawOrientation().rotateBy(getRotationOffset());
    }

    public double getOffsetAccelerationX() {
        return getOffsetAccelerationVector().getX();
    }

    public double getOffsetAccelerationY() {
        return getOffsetAccelerationVector().getY();
    }

    public double getOffsetAccelerationZ() {
        return getOffsetAccelerationVector().getZ();
    }

    public double getRawAccelerationX() {
        return getRawAccelerationVector().getX();
    }

    public double getRawAccelerationY() {
        return getRawAccelerationVector().getY();
    }

    public double getRawAccelerationZ() {
        return getRawAccelerationVector().getZ();
    }

    public double getOffsetPitch() {
        return getOffsetOrientation().getY();
    }

    public double getOffsetYaw() {
        return getOffsetOrientation().getZ();
    }

    public double getOffsetRoll() {
        return getOffsetOrientation().getX();
    }

    public double getRawPitch() {
        return getRawOrientation().getY();
    }

    public double getRawYaw() {
        return getRawOrientation().getZ();
    }

    public double getRawRoll() {
        return getRawOrientation().getX();
    }

    public double getRawPitchVelocity() {
        return unoffsetPitchVFilter.getCurrentOutput();
    }

    public double getRawYawVelocity() {
        return unoffsetYawVFilter.getCurrentOutput();
    }

    public double getRawRollVelocity() {
        return unoffsetRollVFilter.getCurrentOutput();
    }

    public double getOffsetPitchVelocity() {
        return offsetPitchVFilter.getCurrentOutput();
    }

    public double getOffsetYawVelocity() {
        return offsetYawVFilter.getCurrentOutput();
    }

    public double getOffsetRollVelocity() {
        return offsetRollVFilter.getCurrentOutput();
    }

    public double getRawPitchAcceleration() {
        return unoffsetPitchAFilter.getCurrentOutput();
    }

    public double getRawYawAcceleration() {
        return unoffsetYawAFilter.getCurrentOutput();
    }

    public double getRawRollAcceleration() {
        return unoffsetRollAFilter.getCurrentOutput();
    }

    public double getOffsetPitchAcceleration() {
        return offsetPitchAFilter.getCurrentOutput();
    }

    public double getOffsetYawAcceleration() {
        return offsetYawAFilter.getCurrentOutput();
    }

    public double getOffsetRollAcceleration() {
        return offsetRollAFilter.getCurrentOutput();
    }
}
