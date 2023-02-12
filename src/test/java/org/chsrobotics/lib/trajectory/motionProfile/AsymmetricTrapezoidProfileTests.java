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

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import org.chsrobotics.lib.trajectory.motionProfile.AsymmetricTrapezoidProfile.*;
import org.chsrobotics.lib.trajectory.motionProfile.MotionProfile.State;
import org.junit.Test;

/** Tests for the AsymmetricTrapezoidProfile. */
public class AsymmetricTrapezoidProfileTests {
    private final double epsilon = 0.0001;

    AsymmetricTrapezoidProfile positiveTriangleProfile =
            new AsymmetricTrapezoidProfile(
                    new Constraints(10, 1, -2), new State(2, 0), new State(1, 0));

    @Test
    public void AsymmetricTrapezoidProfileConstraintsEqualsWorks() {
        Constraints constraints1 = new Constraints(1, 3, 2);
        Constraints constraints2 = new Constraints(0.99999, 3.00001, 2.00001);
        assertEquals(true, constraints1.equals(constraints2));
    }

    @Test
    public void AsymmetricTrapezoidProfilePositiveTrianglePhases() {
        var expected =
                new ArrayList<>(
                        List.of(
                                new ProfilePhase(1.1547005383792515, 0.6666666666666666, 1.0, 0.0),
                                new ProfilePhase(
                                        0.5773502691896257,
                                        0.3333333333333333,
                                        -2.0,
                                        1.1547005383792515)));
        assertEquals(positiveTriangleProfile.getPhases(), expected);
    }

    @Test
    public void AsymmetricTrapezoidProfilePositiveTriangleTotalTime() {
        assertEquals(positiveTriangleProfile.getMaxReference(), 1.7320508075688772, epsilon);
    }

    @Test
    public void AsymmetricTrapezoidProfilePositiveTriangleCalculate() {
        assertEquals(positiveTriangleProfile.sample(0), new State(1.0, 0.0));
        assertEquals(positiveTriangleProfile.sample(0.5), new State(1.125, 0.5));
        assertEquals(positiveTriangleProfile.sample(1.1), new State(1.605, 1.1));
        assertEquals(
                positiveTriangleProfile.sample(1.5),
                new State(1.9461524227066316, 0.4641016151377544));
        assertEquals(positiveTriangleProfile.sample(2.0), new State(2.0, 0.0));
    }

    AsymmetricTrapezoidProfile negativeTrapezoidProfile =
            new AsymmetricTrapezoidProfile(
                    new Constraints(2, 1, -2), new State(-1, 0), new State(3, 0));

    @Test
    public void AsymmetricTrapezoidProfileNegativeTrapezoidPhases() {
        var expected =
                new ArrayList<>(
                        List.of(
                                new ProfilePhase(2.0, -2.0, -1.0, 0.0),
                                new ProfilePhase(0.5, -1.0, 0.0, -2.0),
                                new ProfilePhase(1.0, -1.0, 2.0, -2.0)));
        assertEquals(negativeTrapezoidProfile.getPhases(), expected);
    }

    @Test
    public void AsymmetricTrapezoidProfileNegativeTrapezoidTotalTime() {
        assertEquals(negativeTrapezoidProfile.getMaxReference(), 3.5, epsilon);
    }

    @Test
    public void AsymmetricTrapezoidProfileNegativeTrapezoidCalculate() {
        assertEquals(negativeTrapezoidProfile.sample(0), new State(3.0, 0.0));
        assertEquals(negativeTrapezoidProfile.sample(0.5), new State(2.875, -0.5));
        assertEquals(negativeTrapezoidProfile.sample(1.1), new State(2.395, -1.1));
        assertEquals(negativeTrapezoidProfile.sample(3.0), new State(-0.75, -1.0));
        assertEquals(negativeTrapezoidProfile.sample(3.5), new State(-1.0, 0.0));
    }

    AsymmetricTrapezoidProfile positiveRampProfile =
            new AsymmetricTrapezoidProfile(
                    new Constraints(10, 1, -2), new State(1, 0), new State(0, 3));

    @Test
    public void AsymmetricTrapezoidProfileRampProfilePhases() {
        var expected =
                new ArrayList<>(List.of(new ProfilePhase(0.6666666666666666, 1.0, -4.5, 3.0)));
        assertEquals(positiveRampProfile.getPhases(), expected);
    }
}
