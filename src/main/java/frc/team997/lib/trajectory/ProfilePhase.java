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

/** A data class for profile phase data. */
public class ProfilePhase {
    public final double time;
    public final double position;
    public final double acceleration;
    public final double initialVelocity;

    /**
     * Constructs a ProfilePhase from a given acceleration, initial velocity, and time. All units of
     * displacement work, as long as they agree.
     *
     * @param acceleration The acceleration throughout this phase (displacement units/time unit/time
     *     unit).
     * @param initialVelocity The velocity at the start of the phase (displacement units/time unit).
     * @param time The duration of the phase (time units).
     * @return A ProfilePhase with the given values and a calculated displacement.
     */
    public ProfilePhase(double acceleration, double initialVelocity, double time) {
        double displacement = (0.5 * (acceleration) * (time * time)) + time * initialVelocity;
        this.time = time;
        this.position = displacement;
        this.acceleration = acceleration;
        this.initialVelocity = initialVelocity;
    }

    /**
     * Constructs a ProfilePhase. All units fine as long as they agree.
     *
     * <p>You probably should use {@code ProfilePhase.fromRatesAndTime()} instead.
     *
     * @param time The duration of the phase (time units).
     * @param position The displacement of the phase (displacement units).
     * @param acceleration The acceleration of the phase (0 if coast phase) (displacement units/time
     *     unit/time unit).
     * @param initialVelocity The velocity at the beginning of a phase (displacement units/time
     *     unit).
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
