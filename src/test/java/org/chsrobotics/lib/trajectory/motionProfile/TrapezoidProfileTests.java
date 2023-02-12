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
package org.chsrobotics.lib.trajectory.motionProfile;

import static org.junit.Assert.assertEquals;

import org.chsrobotics.lib.trajectory.motionProfile.MotionProfile.*;
import org.chsrobotics.lib.trajectory.motionProfile.TrapezoidProfile.Constraints;
import org.junit.Test;

/**
 * Tests for the TrapezoidProfile.
 *
 * <p>Most of this is covered by the AsymmetricTrapezoidProfileTests, but it doesn't hurt to check.
 */
public class TrapezoidProfileTests {
    private final double epsilon = 0.0001;

    @Test
    public void TrapezoidProfileConstraintsEqualityWorks() {
        Constraints constraints1 = new Constraints(1, 5);
        Constraints constraints2 = new Constraints(1.00001, 4.99999);

        assertEquals(true, constraints1.equals(constraints2));
    }

    @Test
    public void TrapezoidProfileAgreesWithAsymmetricTrapezoidProfile() {
        TrapezoidProfile symmetrical =
                new TrapezoidProfile(new Constraints(5, 3), new State(0, 5), new State(40, 2));
        AsymmetricTrapezoidProfile asymmetrical =
                new AsymmetricTrapezoidProfile(
                        new AsymmetricTrapezoidProfile.Constraints(5, 3, 3),
                        new State(0, 5),
                        new State(40, 2));

        assertEquals(symmetrical.getMaxReference(), asymmetrical.getMaxReference(), epsilon);

        for (int i = 0; i < symmetrical.getMaxReference(); i++) {
            assertEquals(true, symmetrical.sample(i).equals(asymmetrical.sample(i)));
        }

        assertEquals(
                true,
                symmetrical
                        .sample(symmetrical.getMaxReference())
                        .equals(asymmetrical.sample(symmetrical.getMaxReference())));

        assertEquals(
                true,
                symmetrical
                        .sample(symmetrical.getMaxReference() + 1)
                        .equals(asymmetrical.sample(symmetrical.getMaxReference() + 1)));
    }
}
