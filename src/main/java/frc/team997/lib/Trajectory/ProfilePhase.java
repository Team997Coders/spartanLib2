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
package frc.team997.lib.Trajectory;

import java.util.Objects;

/**
 * A data class for profile phase data.
 * Used for forming velocity/acceleration profiles with no regard for position accumulation
 */
public class ProfilePhase {
    public final double time;
    public final double position;
    public final double acceleration;
    public final double initialVelocity;

    /**
     * Construct a ProfilePhase.
     *
     * @param time The duration of the phase.
     * @param position The displacement of the phase.
     * @param acceleration The acceleration of the phase (0 if coast phase).
     * @param initialVelocity The velocity at the beginning of a phase.
     */
    public ProfilePhase(double time, double position, double acceleration, double initialVelocity) {
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
        return "Phase[time: "+time+", position: "+position+", acceleration: "+acceleration+", initialVelocity:"+initialVelocity+"]";
    }
}
