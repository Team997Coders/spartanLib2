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
package frc.team997.lib.trajectory;

import static org.junit.Assert.assertEquals;

import frc.team997.lib.trajectory.AsymmetricTrapezoidProfile.*;
import frc.team997.lib.trajectory.MotionProfile.*;
import frc.team997.lib.trajectory.TrapezoidProfile.*;
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
        TrapezoidProfileConstraints constraints1 = new TrapezoidProfileConstraints(1, 5);
        TrapezoidProfileConstraints constraints2 =
                new TrapezoidProfileConstraints(1.00001, 4.99999);

        assertEquals(true, constraints1.equals(constraints2));
    }

    @Test
    public void TrapezoidProfileAgreesWithAsymmetricTrapezoidProfile() {
        TrapezoidProfile symmetrical =
                new TrapezoidProfile(
                        new TrapezoidProfileConstraints(5, 3), new State(0, 5), new State(40, 2));
        AsymmetricTrapezoidProfile asymmetrical =
                new AsymmetricTrapezoidProfile(
                        new AsymmetricTrapezoidProfileConstraints(5, 3, 3),
                        new State(0, 5),
                        new State(40, 2));

        assertEquals(symmetrical.totalTime(), asymmetrical.totalTime(), epsilon);

        for (int i = 0; i < symmetrical.totalTime(); i++) {
            assertEquals(true, symmetrical.calculate(i).equals(asymmetrical.calculate(i)));
        }

        assertEquals(
                true,
                symmetrical
                        .calculate(symmetrical.totalTime())
                        .equals(asymmetrical.calculate(symmetrical.totalTime())));

        assertEquals(
                true,
                symmetrical
                        .calculate(symmetrical.totalTime() + 1)
                        .equals(asymmetrical.calculate(symmetrical.totalTime() + 1)));
    }
}
