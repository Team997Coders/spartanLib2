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

import java.util.Objects;

/** Holds constant-acceleration kinematic rates and time. */
public class ProfilePhase {
    /** Time to complete the phase. */
    public final double time;

    /** Change in position through the phase. */
    public final double position;

    /** Acceleration through the phase. */
    public final double acceleration;

    /** Velocity of the phase at its start. */
    public final double initialVelocity;

    /**
     * Constructs a ProfilePhase from a given acceleration, initial velocity, and time.
     *
     * @param acceleration The acceleration throughout this phase.
     * @param initialVelocity The velocity at the start of the phase.
     * @param time The duration of the phase.
     */
    public ProfilePhase(double acceleration, double initialVelocity, double time) {
        double displacement = (0.5 * (acceleration) * (time * time)) + time * initialVelocity;
        this.time = time;
        this.position = displacement;
        this.acceleration = acceleration;
        this.initialVelocity = initialVelocity;
    }

    /**
     * Constructs a ProfilePhase.
     *
     * @param time The duration of the phase.
     * @param position The displacement of the phase.
     * @param acceleration The acceleration of the phase (0 if coast phase).
     * @param initialVelocity The velocity at the beginning of a phase.
     */
    protected ProfilePhase(
            double time, double position, double acceleration, double initialVelocity) {
        this.time = time;
        this.position = position;
        this.acceleration = acceleration;
        this.initialVelocity = initialVelocity;
    }

    @Override
    public boolean equals(Object other) {
        double epsilon = 0.0001;
        if (other instanceof ProfilePhase) {
            ProfilePhase rhs = (ProfilePhase) other;
            return this.time == rhs.time
                    && Math.abs(this.position - rhs.position) < epsilon
                    && Math.abs(this.acceleration - rhs.acceleration) < epsilon
                    && Math.abs(this.initialVelocity - rhs.initialVelocity) < epsilon;
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(time, position, acceleration, initialVelocity);
    }

    @Override
    public String toString() {
        return "Phase[time: "
                + time
                + ", position: "
                + position
                + ", acceleration: "
                + acceleration
                + ", initialVelocity:"
                + initialVelocity
                + "]";
    }
}
