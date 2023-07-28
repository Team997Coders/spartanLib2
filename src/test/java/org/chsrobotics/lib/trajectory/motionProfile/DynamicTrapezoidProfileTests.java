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
import org.chsrobotics.lib.trajectory.motionProfile.DynamicTrapezoidProfile.*;
import org.junit.Test;

/** Tests for the AsymmetricTrapezoidProfile. */
public class DynamicTrapezoidProfileTests {
    DynamicTrapezoidProfile basicProfile =
            new DynamicTrapezoidProfile(new DynamicTrapezoidProfileConstraints(10, 1));

    private void assertMotionProfileAsExpected(
            DynamicTrapezoidProfile profile,
            MotionProfileState initial,
            double targetPosition,
            double controlPeriod,
            List<MotionProfileState> expectedStates) {
        MotionProfileState state = profile.getNextSetpoint(targetPosition, initial, controlPeriod);
        for (MotionProfileState expectedState : expectedStates) {
            assertEquals(
                    "Assert failure between motion profile states: expected("
                            + expectedState.position()
                            + ","
                            + expectedState.velocity()
                            + "), actual("
                            + state.position()
                            + ","
                            + state.velocity()
                            + ")",
                    expectedState,
                    state);

            state = profile.getNextSetpoint(targetPosition, state, controlPeriod);
        }
    }

    @Test
    public void TrapezoidProfileStaticPositiveTarget() {
        ArrayList<MotionProfileState> expected = new ArrayList<>();
        expected.add(new MotionProfileState(0.5, 1));
        expected.add(new MotionProfileState(2, 2));
        expected.add(new MotionProfileState(3.5, 1));
        expected.add(new MotionProfileState(4, 0));
        expected.add(new MotionProfileState(4.5, 1));
        expected.add(new MotionProfileState(5, 0));
        expected.add(new MotionProfileState(5, 0));

        assertMotionProfileAsExpected(basicProfile, new MotionProfileState(0, 0), 5, 1, expected);
    }

    @Test
    public void TrapezoidProfileNegativeTarget() {
        ArrayList<MotionProfileState> expected = new ArrayList<>();
        expected.add(new MotionProfileState(3.5, -1));
        expected.add(new MotionProfileState(2, -2));
        expected.add(new MotionProfileState(-0.5, -3));
        expected.add(new MotionProfileState(0, 0));

        assertMotionProfileAsExpected(basicProfile, new MotionProfileState(4, 0), -3, 1, expected);
    }
}
