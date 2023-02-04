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
package org.chsrobotics.lib.controllers.feedforward;

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
    private final double segALength;

    private final double segACoMRadius;
    private final double segBCoMRadius;

    private final double segAMass;
    private final double segBMass;

    /**
     * Constructs a TwoJointArmFeedforward.
     *
     * @param segACoMRadius The distance from joint A to the center of mass of the directly attached
     *     segment, in meters.
     * @param segAMass The mass of segment A (the segment closest to the root), in kilograms.
     * @param segBCoMRadius The distance from joint B to the center of mass of the directly attached
     *     segment, in meters.
     * @param segBMass The mass of segment B (the segment closest to the end effector), in
     *     kilograms.
     */
    public TwoJointArmFeedforward(
            double segACoMRadius,
            double segAMass,
            double segALength,
            double segBCoMRadius,
            double segBMass) {
        this.segALength = segALength;

        this.segACoMRadius = segACoMRadius;
        this.segAMass = segAMass;

        this.segBCoMRadius = segBCoMRadius;
        this.segBMass = segBMass;
    }

    /**
     * @param jointAngles
     * @param jointVelocities
     * @return
     */
    public Tuple2<Double> getFeedforwardTorques(
            Tuple2<Double> jointAngles, Tuple2<Double> jointVelocities) {
        double jointACoriolisTorque = 0; // TODO

        double jointBCoriolisTorque =
                jointVelocities.secondValue()
                        * jointVelocities.firstValue()
                        * segBMass
                        * segALength
                        * segBCoMRadius
                        * Math.sin(jointAngles.firstValue());

        double jointAPartialGravityTorque =
                (((segAMass * segACoMRadius) + (segBMass * segBCoMRadius))
                        * (-9.81)
                        * Math.cos(jointAngles.firstValue()));

        double jointBGravityTorque =
                segBMass
                        * segBCoMRadius
                        * -9.81
                        * Math.cos(jointAngles.firstValue() + jointAngles.secondValue());

        double jointATorque =
                jointACoriolisTorque + jointAPartialGravityTorque + jointBGravityTorque;
        double jointBTorque = jointBCoriolisTorque + jointBGravityTorque;

        return Tuple2.of(jointATorque, jointBTorque);
    }
}
