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
package org.chsrobotics.lib.controllers;

import org.chsrobotics.lib.controllers.PID.PIDConstants;
import org.chsrobotics.lib.trajectory.MotionProfile;

/**
 * A PID controller with built-in motion profile following. The setpoint of the controller at a time
 * is the position value of the motion profile at that time.
 *
 * <p>For more information on PID controllers and motion profiles, see {@link
 * org.chsrobotics.lib.controllers.PID}, {@link org.chsrobotics.lib.trajectory.MotionProfile}, and
 * {@link org.chsrobotics.lib.trajectory.AsymmetricTrapezoidProfile} and {@link
 * org.chsrobotics.lib.trajectory.TrapezoidProfile}.
 */
public class MotionProfilePID {
    private final PID controller;

    private final MotionProfile profile;

    private double reference;

    private double lastReference = 0;

    /**
     * Constructs a MotionProfilePID out of provided PIDConstants and a MotionProfile to follow.
     *
     * @param constants The gains of the PID controller.
     * @param profile The motion profile for the setpoint of the controller to follow.
     */
    public MotionProfilePID(PID.PIDConstants constants, MotionProfile profile) {
        controller = new PID(constants, profile.calculate(0).position);
        this.profile = profile;
    }

    /**
     * Constructs a MotionProfilePID out of kP, kI, and kD gains, and a MotionProfile to follow.
     *
     * @param kP The Proportional gain of the controller.
     * @param kI The Integral gain of the controller.
     * @param kD The Derivative gain of the controller.
     * @param profile The motion profile for the setpoint of the controller to follow.
     */
    public MotionProfilePID(double kP, double kI, double kD, MotionProfile profile) {
        controller = new PID(kP, kI, kD, profile.calculate(0).position);
        this.profile = profile;
    }

    /**
     * Sets the gains of the controller using provided gains.
     *
     * @param kP The proportional gain for the controller.
     * @param kI The integral gain for the controller.
     * @param kD The derivative gain for the controller.
     */
    public void setConstants(double kP, double kI, double kD) {
        controller.setConstants(kP, kI, kD);
    }

    /**
     * Returns the current gains of the controller.
     *
     * @return A PIDConstants containing the gains of the controller.
     */
    public PIDConstants getConstants() {
        return controller.getConstants();
    }

    /**
     * Sets the gains of the controller using a provided PIDConstants.
     *
     * @param constants The PIDConstants containing the gains for the controller.
     */
    public void setConstants(PIDConstants constants) {
        controller.setConstants(constants);
    }

    /**
     * Returns the current proportional gain of the controller.
     *
     * @return The current kP.
     */
    public double getKP() {
        return controller.getkP();
    }

    /**
     * Sets the proportional gain of the controller.
     *
     * @param kP The new kP for the controller.
     */
    public void setKP(double kP) {
        controller.setkP(kP);
    }

    /**
     * Returns the current integral gain of the controller.
     *
     * @return The current kI.
     */
    public double getKI() {
        return controller.getkI();
    }

    /**
     * Sets the integral gain of the controller.
     *
     * @param kI The new kI for the controller.
     */
    public void setKI(double kI) {
        controller.setkI(kI);
    }

    /**
     * Returns the current derivative gain of the controller.
     *
     * @return The current kD.
     */
    public double getKD() {
        return controller.getkD();
    }

    /**
     * Sets the derivative gain of the controller.
     *
     * @param kD The new kD for the controller.
     */
    public void setKD(double kD) {
        controller.setkD(kD);
    }

    /**
     * Returns the current setpoint (target) of the controller, as a State with both position and
     * velocity.
     *
     * @param time The time into the profile to sample, above zero in seconds.
     * @return A State representing the motion profile at that time.
     */
    public MotionProfile.State getSetpoint(double time) {
        return profile.calculate(time);
    }

    /**
     * Returns the current setpoint (target) of the controller, as a State with both position and
     * velocity.
     *
     * <p>This uses the last value of {@code reference} given to this class's {@code calculate()}
     * method.
     *
     * @return A State representing the motion profile at that time.
     */
    public MotionProfile.State getSetpoint() {
        return profile.calculate(reference);
    }

    /**
     * Returns the accumulated past error in the integral term. Not equal to the output of the I
     * term: this is not multiplied by the gain.
     *
     * @return The integral of error with respect to time from the last reset to now.
     */
    public double getIntegralAccumulation() {
        return controller.getIntegralAccumulation();
    }

    /** Resets accumulation of past error in the integral term. */
    public void resetIntegralAccumulation() {
        controller.resetIntegralAccumulation();
    }

    /** Resets the previous measurement used for velocity approximation for the derivative term. */
    public void resetPreviousMeasurement() {
        controller.resetPreviousMeasurement();
    }

    /** Resets all references to past states in the controller, effectively restarting it. */
    public void reset() {
        controller.reset();
        lastReference = 0;
        reference = 0;
    }

    /**
     * Sets the maximum position allowed for {@code isFinished()} to return true.
     *
     * @param positionTolerance The maximum allowed position error, as a proportion of the setpoint.
     */
    public void setSetpointTolerance(double positionTolerance) {
        controller.setSetpointTolerance(positionTolerance, 0);
    }

    /**
     * Returns the maximum allowed position error for {@code isAtSetpoint()} to return true.
     *
     * @return The maximum allowed position error, as a proportion of the setpoint.
     */
    public double getSetpointPositionTolerance() {
        return controller.getSetpointPositionTolerance();
    }

    /**
     * Returns the last reported measurement given to the controller.
     *
     * @return The last reported measurement (0 if none have been given).
     */
    public double getCurrentState() {
        return controller.getCurrentState();
    }

    /**
     * Returns an output from the controller, provided a feedback measurement and place into the
     * MotionProfile to sample.
     *
     * <p>For this to work, the value of {@code reference} must increment by the same amount of
     * times between calls of this.
     *
     * @param measurement The value of the measured feedback.
     * @param reference The time, in seconds greater than zero, to sample into the profile.
     * @return The sum of the P, I, and D terms.
     */
    public double calculate(double measurement, double reference) {
        controller.setSetpoint(profile.calculate(reference).position);
        double effort = controller.calculate(measurement, reference - lastReference);
        lastReference = reference;
        return effort;
    }

    /**
     * Returns an output from the controller, provided a feedback measurement, place into the
     * MotionProfile, and delta-time.
     *
     * @param measurement The value of the measured feedback.
     * @param reference The time, in seconds greater than zero, to sample into the profile.
     * @param dt The elapsed time since the last call of any of this class's {@code calculate()}
     *     methods.
     * @return The sum of the P, I, and D terms.
     */
    public double calculate(double measurement, double reference, double dt) {
        controller.setSetpoint(profile.calculate(reference).position);
        double effort = controller.calculate(measurement, dt);
        lastReference = reference;
        return effort;
    }

    /**
     * Returns whether the controller has reached the position of the setpoint.
     *
     * @return Whether the controller is within the minimum position error.
     */
    public boolean isAtSetpoint() {
        return (Math.abs(
                        profile.calculate(profile.totalTime()).position
                                - controller.getCurrentState())
                < Math.abs(
                        controller.getSetpointPositionTolerance()
                                * profile.calculate(profile.totalTime()).position));
    }
}
