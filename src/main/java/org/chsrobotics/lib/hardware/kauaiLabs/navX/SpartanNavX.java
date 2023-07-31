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
package org.chsrobotics.lib.hardware.kauaiLabs.navX;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.datalog.DataLog;
import org.chsrobotics.lib.hardware.base.imu.AbstractIMU;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

// TODO: docs
public class SpartanNavX extends AbstractIMU {
    public static record NavXConfig(Rotation3d offset, boolean isReal) {
        public NavXConfig setOffset(Rotation3d offset) {
            return new NavXConfig(offset, isReal);
        }

        public NavXConfig setIsReal(boolean isReal) {
            return new NavXConfig(offset, isReal);
        }
    }

    private final NavXConfig config;

    private Logger<Double> temperatureLogger;
    private Logger<Boolean> isCalibratingLogger;

    private boolean logsConstructed = false;

    private AHRS navx;

    public SpartanNavX(NavXConfig config) {
        this.config = config;

        PeriodicCallbackHandler.registerCallback(this::updateLogs);
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            super.autoGenerateLogs(log, name, subdirName, publishToNT, recordInLog);

            temperatureLogger =
                    new Logger<>(
                            log, name + "/temperature_c", subdirName, publishToNT, recordInLog);

            isCalibratingLogger =
                    new Logger<>(
                            log, name + "/isCalibrating", subdirName, publishToNT, recordInLog);

            PeriodicCallbackHandler.registerCallback(this::updateLogs);

            logsConstructed = true;
        }
    }

    public NavXConfig getConfig() {
        return config;
    }

    @Override
    public Rotation3d getRotationOffset() {
        return config.offset;
    }

    @Override
    public Translation3d getRawAccelerationVector() {
        final double g = 9.81;

        // TODO: ensure axes align as expected on physical hardware

        if (config.isReal)
            return new Translation3d(
                    g * navx.getRawAccelX(), g * navx.getRawAccelY(), g * navx.getRawAccelZ());
        else return new Translation3d();
    }

    @Override
    public Rotation3d getRawOrientation() {
        if (config.isReal) {
            // navx api is downright psychotic
            double qW = -Math.PI * navx.getQuaternionW();
            double qX = -Math.PI * navx.getQuaternionX();
            double qY = -Math.PI * navx.getQuaternionY();
            double qZ = -Math.PI * navx.getQuaternionZ();

            // TODO: check to make sure this aligns with actual hardware
            return new Rotation3d(new Quaternion(qW, qX, qY, qZ));
        } else return new Rotation3d();
    }

    public boolean isCalibrating() {
        if (config.isReal) return navx.isCalibrating();
        else return false;
    }

    public void startCalibration() {
        if (config.isReal) navx.calibrate();
    }

    private void updateLogs() {
        if (logsConstructed) {
            if (config.isReal) {
                temperatureLogger.update((double) navx.getTempC());
            }

            isCalibratingLogger.update(isCalibrating());
        }
    }

    @Override
    public boolean isStale() {
        if (config.isReal) {
            return !navx.isConnected();
        } else return false;
    }
}
