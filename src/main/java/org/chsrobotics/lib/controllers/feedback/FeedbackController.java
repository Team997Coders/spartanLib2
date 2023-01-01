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
package org.chsrobotics.lib.controllers.feedback;

/** Interface for the feedback (past state-dependent and -varying) controllers of the library. */
public interface FeedbackController {
    public final double defaultPositionErrorToleranceProportion = 0.02;
    public final double defaultVelocityErrorToleranceProportion = 0.02;

    /** Resets the controller. */
    void reset();

    /**
     * Sets the setpoint (target) of the controller.
     *
     * @param newSetpoint The new target of the controller.
     */
    void setSetpoint(double newSetpoint);

    /**
     * Returns the current setpoint (target) of the controller.
     *
     * @return The current setpoint.
     */
    double getSetpoint();

    /**
     * Sets the maximum error of both position and velocity allowed for {@code atSetpoint()} to
     * return true.
     *
     * @param positionErrorProportion The maximum allowed position error, as a proportion of the
     *     setpoint.
     * @param velocityErrorProportion The maximum allowed absolute velocity per second, as a
     *     proportion of the setpoint / second.
     */
    void setSetpointTolerance(double positionErrorProportion, double velocityErrorProportion);

    /**
     * Returns an output from the controller with the default robot loop time.
     *
     * <p>This must be called at a rate of once every robot loop to be consistent.
     *
     * @param measurement The value of the measured feedback.
     * @return The output of the controller.
     */
    double calculate(double measurement);

    /**
     * Returns an output from the controller with a given dt.
     *
     * @param measurement The value of the measured feedback.
     * @param dt The time, in seconds, since the last update of this controller.
     * @return The output of the controller.
     */
    double calculate(double measurement, double dt);

    /**
     * Returns the last output of the controller without updating with a new value.
     *
     * @return The last ouptut of the controller.
     */
    double getCurrentValue();

    /**
     * Returns whether the controller has reached the setpoint with minimal velocity.
     *
     * @return Whether the controller is within the minimum position and velocity errors.
     */
    boolean atSetpoint();
}
