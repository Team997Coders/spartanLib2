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

import java.util.Objects;

/**
 * Represents a special case of an {@link AsymmetricTrapezoidProfile} where the maxAcceleration and
 * maxDeceleration constraints are the same.
 *
 * <p>A motion profile that allows for smooth motion between two states. The dx/dt of this motion
 * (velocity) forms the trapezoid shape on a graph. The particular elements of motion are velocity
 * and acceleration limited.
 *
 * <p>The most useful practical application of this is use defining the setpoint for a PID (or
 * similar) controller, to avoid control effort saturation or the initial spike in input from the P
 * term.
 *
 * <p>Also could be used to meet physical constraints from a mechanism (can't go too fast or the
 * wires might tangle, mechanism has too much mass to decelerate too quickly, etc...).
 */
public class TrapezoidProfile extends AsymmetricTrapezoidProfile {

    /** Data class to hold the maximum allowed rates for the output of a TrapezoidProfile. */
    public static class TrapezoidProfileConstraints {
        public final double maxVelocity;
        public final double maxAcceleration;

        /**
         * Constructs a TrapezoidProfileConstraints.
         *
         * @param maxVelocity maximum allowed velocity.
         * @param maxAcceleration maximum allowed acceleration (both directions).
         */
        public TrapezoidProfileConstraints(double maxVelocity, double maxAcceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }

        @Override
        public boolean equals(Object other) {
            double epsilon = 0.0001;
            if (other instanceof TrapezoidProfileConstraints) {
                TrapezoidProfileConstraints rhs = (TrapezoidProfileConstraints) other;
                return Math.abs(this.maxVelocity - rhs.maxVelocity) < epsilon
                        && Math.abs(this.maxAcceleration - rhs.maxAcceleration) < epsilon;
            } else {
                return false;
            }
        }

        @Override
        public int hashCode() {
            return Objects.hash(maxVelocity, maxAcceleration);
        }

        @Override
        public String toString() {
            return "TrapezoidProfileConstraints[maxVelocity: "
                    + maxVelocity
                    + ", maxAcceleration: "
                    + maxAcceleration
                    + "]";
        }
    }

    /**
     * Constructs a TrapezoidProfile out of given constraints, an initial state, and a goal state.
     *
     * @param constraints The maximum allowed velocity and accelerations.
     * @param initialState The initial state of the profile.
     * @param goalState The target state of the profile.
     */
    public TrapezoidProfile(
            TrapezoidProfileConstraints constraints, State goalState, State initialState) {
        super(
                new AsymmetricTrapezoidProfileConstraints(
                        constraints.maxVelocity,
                        constraints.maxAcceleration,
                        constraints.maxAcceleration),
                goalState,
                initialState);
    }

    /**
     * Constructs a Trapezoid profile out of given constraints, a goal state, and an initial state
     * of zero position and velocity.
     *
     * @param constraints The maximum allowed velocity and accelerations.
     * @param goalState The target state of the profile.
     */
    public TrapezoidProfile(TrapezoidProfileConstraints constraints, State goalState) {
        super(
                new AsymmetricTrapezoidProfileConstraints(
                        constraints.maxVelocity,
                        constraints.maxAcceleration,
                        constraints.maxAcceleration),
                goalState);
    }
}
