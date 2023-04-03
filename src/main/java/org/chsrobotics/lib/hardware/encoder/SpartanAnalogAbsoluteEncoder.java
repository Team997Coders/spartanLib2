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

import edu.wpi.first.util.datalog.DataLog;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public class SpartanAnalogAbsoluteEncoder extends AbstractAbsoluteEncoder {

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        PeriodicCallbackHandler.registerCallback(this::updateLogs);
        throw new UnsupportedOperationException("Unimplemented method 'autoGenerateLogs'");
    }

    private void updateLogs(double dtSeconds) {
        throw new UnsupportedOperationException("Unimplemented method 'updateLogs'");
    }

    @Override
    public double getRawPosition() {
        throw new UnsupportedOperationException("Unimplemented method 'getRawPosition'");
    }

    @Override
    public double getRawVelocity() {
        throw new UnsupportedOperationException("Unimplemented method 'getRawVelocity'");
    }

    @Override
    public double getRawAcceleration() {
        throw new UnsupportedOperationException("Unimplemented method 'getRawAcceleration'");
    }

    @Override
    public double getConvertedPosition() {
        throw new UnsupportedOperationException("Unimplemented method 'getConvertedPosition'");
    }

    @Override
    public double getConvertedVelocity() {
        throw new UnsupportedOperationException("Unimplemented method 'getConvertedVelocity'");
    }

    @Override
    public double getConvertedAcceleration() {
        throw new UnsupportedOperationException("Unimplemented method 'getConvertedAcceleration'");
    }

    @Override
    public double getOffset() {
        throw new UnsupportedOperationException("Unimplemented method 'getOffset'");
    }

    @Override
    public double getUnoffsetConvertedPosition() {
        throw new UnsupportedOperationException(
                "Unimplemented method 'getUnoffsetConvertedPosition'");
    }

    @Override
    public double getUnoffsetRawPosition() {
        throw new UnsupportedOperationException("Unimplemented method 'getUnoffsetRawPosition'");
    }

    @Override
    public double getUnitsPerCount() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getUnitsPerCount'");
    }

    @Override
    public boolean getInverted() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
    }

    @Override
    public double getRawCounts() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRawCounts'");
    }
}
