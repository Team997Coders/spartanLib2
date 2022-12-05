/**
Copyright 2022 FRC Team 997

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
package org.chsrobotics.lib.drive.differential;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class DifferentialDriveInputTests {
    @Test
    public void differentialMoveAdd() {
        DifferentialDrivetrainInput actual =
                new DifferentialDrivetrainInput(0.5, 0.5)
                        .add(new DifferentialDrivetrainInput(0.5, -0.2));
        DifferentialDrivetrainInput expected = new DifferentialDrivetrainInput(1.0, 0.3);
        assertEquals(actual, expected);
    }

    @Test
    public void differentialMoveMultiply() {
        DifferentialDrivetrainInput actual =
                new DifferentialDrivetrainInput(1.0, 0.0).multiply(-0.5);
        DifferentialDrivetrainInput expected = new DifferentialDrivetrainInput(-0.5, 0.0);
        assertEquals(actual, expected);
    }
}
