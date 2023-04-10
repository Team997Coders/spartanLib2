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

public class SpartanAnalogAbsoluteEncoder extends AbstractAbsoluteEncoder {

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
        throw new UnsupportedOperationException("Unimplemented method 'getUnitsPerCount'");
    }

    @Override
    public boolean getInverted() {
        throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
    }

    @Override
    public double getRawCounts() {
        throw new UnsupportedOperationException("Unimplemented method 'getRawCounts'");
    }
}
