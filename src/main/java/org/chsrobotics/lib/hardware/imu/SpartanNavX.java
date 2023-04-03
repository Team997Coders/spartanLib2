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
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public class SpartanNavX extends AbstractIMU {
    public static record NavXConfig(Rotation3d offset) {}

    private final NavXConfig config;

    private Logger<Double> temperatureLogger;
    private Logger<Boolean> isCalibratingLogger;

    private boolean logsConstructed = false;

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
        throw new UnsupportedOperationException("Unimplemented method 'getRawAccelerationVector'");
    }

    @Override
    public Rotation3d getRawOrientation() {
        throw new UnsupportedOperationException("Unimplemented method 'getRawOrientation'");
    }

    public boolean isCalibrating() {
        return false;
    }

    public void startCalibration() {}

    private void updateLogs(double dtSeconds) {
        if (logsConstructed) {
            temperatureLogger.update(null);
            isCalibratingLogger.update(null);
        }
    }
}
