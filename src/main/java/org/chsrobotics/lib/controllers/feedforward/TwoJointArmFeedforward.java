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
package org.chsrobotics.lib.controllers.feedforward;

import org.chsrobotics.lib.kinematics.RevoluteRevoluteKinematics.RRConfiguration;
import org.chsrobotics.lib.util.Tuple2;

/**
 * Feedforward for a arm with 2 revolute (rotational) degrees of freedom.
 *
 * <p>Should be used in conjunction with {@link
 * edu.wpi.first.math.controller.SimpleMotorFeedforward} such that each motor has another
 * feedforward controller of its own, in addition to this one. The control efforts of the
 * feedforward controllers (and additional feedback) should be summed.
 */
public class TwoJointArmFeedforward {
    private final double kTJointA;
    private final double kTJointB;

    private double segACoMRadius;
    private double segBCoMRadius;

    private double segAMass;
    private double segBMass;

    /**
     * Constructs a TwoJointArmFeedforward.
     *
     * @param kTJointA Torque gain for joint A (the joint at the root).
     * @param kTJointB Torque gain for joint B (the joint between arm segments).
     * @param segACoMRadius The distance from joint A to the center of mass of the directly attached
     *     segment, in meters.
     * @param segAMass The mass of segment A (the segment closest to the root), in kilograms.
     * @param segBCoMRadius The distance from joint B to the center of mass of the directly attached
     *     segment, in meters.
     * @param segBMass The mass of segment B (the segment closest to the end effector), in
     *     kilograms.
     */
    public TwoJointArmFeedforward(
            double kTJointA,
            double kTJointB,
            double segACoMRadius,
            double segAMass,
            double segBCoMRadius,
            double segBMass) {
        this.kTJointA = kTJointA;

        this.kTJointB = kTJointB;

        this.segACoMRadius = segACoMRadius;

        this.segAMass = segAMass;

        this.segBCoMRadius = segBCoMRadius;
        this.segBMass = segBMass;
    }

    /**
     * Calculates the result of the controller, with provided torques on each arm joint. Tangental
     * forces should be counterclockwise-positive, or in other words, the torque should be expressed
     * such that Z+ is positive.
     *
     * <p>If gravity as well as other torques are acting upon the arm, for each joint you can sum
     * the results of this (not calculated with gravity) and {@code gravityTorqueCalculate()}.
     *
     * @param jointATorque The torque on joint A, in newton-meters.
     * @param jointBTorque The torque on joint B, in newton-meters.
     * @return A tuple of control efforts. The first value is the result for joint A, the second
     *     value the result for joint B.
     */
    public Tuple2<Double> overrideTorqueCalculate(double jointATorque, double jointBTorque) {
        return Tuple2.of(kTJointA * jointATorque, kTJointB * jointBTorque);
    }

    /**
     * Calculates the outputs of the controller, using gravity and the masses of the segments to
     * determine the torque on each joint.
     *
     * @param configuration Configuration of the arm, whcih includes the arm angles used for torque
     *     calculation.
     * @return A tuple of control efforts. The first value is the result for joint A, the second
     *     value the result for joint B.
     */
    public Tuple2<Double> gravityTorqueCalculate(RRConfiguration configuration) {
        // our reference frame is oriented such that up is +, so gravities should be negative
        double jointBGravityTorque =
                Math.cos(configuration.endEffectorAngle) * segBMass * -9.81 * segBCoMRadius;

        double jointAGravityTorque =
                Math.cos(configuration.jointAAngle) * segAMass * -9.81 * segACoMRadius;

        // we are treating the entire arm as a rigid body momentarily, so torque can be summed
        return overrideTorqueCalculate(
                jointAGravityTorque + jointBGravityTorque, jointBGravityTorque);
    }

    /**
     * Sets new mass parameters for arm segment A (the segment closest to the root).
     *
     * @param segAMass The new mass of the arm segment, in kilograms.
     * @param segACoMRadius The distance from joint A (the joint attached to the root) to the center
     *     of mass of segment A, in meters.
     */
    public void setSegmentAMassParams(double segAMass, double segACoMRadius) {
        this.segAMass = segAMass;
        this.segACoMRadius = segACoMRadius;
    }

    /**
     * Sets new mass parameters for arm segment B (the segment closest to the end effector).
     *
     * @param segBMass The new mass of the arm segment, in kilograms.
     * @param segBCoMRadius The distance from joint B (the joint between segments) to the center of
     *     mass of segment B, in meters.
     */
    public void setSegmentBMassParams(double segBMass, double segBCoMRadius) {
        this.segBMass = segBMass;
        this.segBCoMRadius = segBCoMRadius;
    }
}
