/**
Copyright 2022-2023 FRC Team 997

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

import java.lang.reflect.Constructor;
import java.util.Map;
import java.util.function.Supplier;
import org.chsrobotics.lib.input.JoystickAxis;
import org.junit.Test;

public class DifferentialDriveModeTests {
    @SuppressWarnings("unchecked")
    JoystickAxis constructTestAxis(double value) {
        Constructor<JoystickAxis> constructor =
                (Constructor<JoystickAxis>) JoystickAxis.class.getDeclaredConstructors()[0];
        constructor.setAccessible(true);
        try {
            return constructor.newInstance((Supplier<Double>) () -> value, "TEST AXIS", false);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Test
    public void tankDriveExecute() {
        JoystickAxis left = constructTestAxis(0.5);
        JoystickAxis right = constructTestAxis(0.25);
        DifferentialDriveMode drive = new TankDrive(left::getValue, right::getValue, () -> 1.0, 0);
        assertEquals(drive.execute(), new DifferentialDrivetrainInput(0.5, 0.25));
    }

    @Test
    public void arcadeDriveExecute() {
        JoystickAxis linear = constructTestAxis(0.5);
        JoystickAxis rotational = constructTestAxis(0.25);
        DifferentialDriveMode drive =
                new ArcadeDrive(linear::getValue, rotational::getValue, () -> 1.0, () -> 1.0, 0, 0);
        assertEquals(drive.execute(), new DifferentialDrivetrainInput(0.75, 0.25));
    }

    @Test
    public void curvatureDriveExecute() {
        JoystickAxis linear = constructTestAxis(0.5);
        JoystickAxis rotational = constructTestAxis(0.25);
        DifferentialDriveMode drive =
                new CurvatureDrive(
                        linear::getValue, rotational::getValue, () -> 1.0, () -> 1.0, 0, 0, false);
        assertEquals(drive.execute(), new DifferentialDrivetrainInput(0.625, 0.375));
    }

    @Test
    public void mixedCurvatureArcadeDriveExecute() {
        JoystickAxis linear = constructTestAxis(0.5);
        JoystickAxis rotational = constructTestAxis(0.25);
        DifferentialDriveMode drive =
                new MixedDrive(
                        Map.of(
                                new ArcadeDrive(
                                                linear::getValue,
                                                rotational::getValue,
                                                () -> 1.0,
                                                () -> 1.0,
                                                0,
                                                0),
                                        0.5,
                                new CurvatureDrive(
                                                linear::getValue,
                                                rotational::getValue,
                                                () -> 1.0,
                                                () -> 1.0,
                                                0,
                                                0,
                                                true),
                                        0.5));
        assertEquals(drive.execute(), new DifferentialDrivetrainInput(0.6875, 0.3125));
    }
}
