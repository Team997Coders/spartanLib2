/**
Copyright 2023 FRC Team 997

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
package org.chsrobotics.lib.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import org.chsrobotics.lib.controllers.feedforward.TwoJointArmFeedforward;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.math.filters.IntegratingFilter;
import org.chsrobotics.lib.util.Tuple2;

public class DoubleJointedArmSim {
    private final DCMotor localGearbox;
    private final DCMotor distalGearbox;

    private final TwoJointArmFeedforward model;

    private final IntegratingFilter localVFilter = new IntegratingFilter(0);
    private final IntegratingFilter localPFilter = new IntegratingFilter(0);

    private final IntegratingFilter distalVFilter = new IntegratingFilter(0);
    private final IntegratingFilter distalPFilter = new IntegratingFilter(0);

    private double localVoltageInput = 0;
    private double distalVoltageInput = 0;

    private final double localMoment;
    private final double distalMoment;

    private final double segAKFriction;
    private final double segBKFriction;

    /**
     * Constructs a DoubleJointedArmSim.
     *
     * @param localGearbox TODO
     * @param distalGearbox
     * @param segACoMRadius The distance from joint A to the center of mass of the directly attached
     *     segment, in meters.
     * @param segAMass The mass of segment A (the segment closest to the root), in kilograms.
     * @param segACoMMoment
     * @param segBCoMRadius The distance from joint B to the center of mass of the directly attached
     *     segment, in meters.
     * @param segBMass The mass of segment B (the segment closest to the end effector), in
     *     kilograms.
     * @param segBCoMMoment
     * @param segAKFriction
     * @param segBKFriction
     */
    public DoubleJointedArmSim(
            DCMotor localGearbox,
            DCMotor distalGearbox,
            double segACoMRadius,
            double segAMass,
            double segACoMMoment,
            double segALength,
            double segBCoMRadius,
            double segBMass,
            double segBCoMMoment,
            double segAKFriction,
            double segBKFriction) {
        this.localGearbox = localGearbox;
        this.distalGearbox = distalGearbox;
        model =
                new TwoJointArmFeedforward(
                        segACoMRadius, segAMass, segALength, segBCoMRadius, segBMass);
        localMoment = segACoMMoment;
        distalMoment = segBCoMMoment;

        this.segAKFriction = segAKFriction;
        this.segBKFriction = segBKFriction;
    }

    public void setInputs(double localVoltage, double distalVoltage) {
        localVoltageInput = localVoltage;
        distalVoltageInput = distalVoltage;
    }

    public void update(double dtSeconds) {
        var torques =
                model.getFeedforwardTorques(
                        Tuple2.of(
                                localPFilter.getCurrentOutput(), distalPFilter.getCurrentOutput()),
                        Tuple2.of(
                                localVFilter.getCurrentOutput(), distalVFilter.getCurrentOutput()));

        double localFriction = -Math.signum(localVFilter.getCurrentOutput()) * segAKFriction;

        double distalFriction = -Math.signum(distalVFilter.getCurrentOutput()) * segBKFriction;

        double localTorqueSupplied =
                ((localGearbox.stallTorqueNewtonMeters / localGearbox.stallCurrentAmps)
                                / localGearbox.rOhms)
                        * localVoltageInput;

        double distalTorqueSupplied =
                ((distalGearbox.stallTorqueNewtonMeters / localGearbox.stallCurrentAmps)
                                / distalGearbox.rOhms)
                        * distalVoltageInput;

        double localA = (torques.firstValue() + localFriction - localTorqueSupplied) / localMoment;

        double distalA =
                (torques.secondValue() + distalFriction - distalTorqueSupplied) / distalMoment;

        localPFilter.calculate(localVFilter.calculate(localA), dtSeconds);

        distalPFilter.calculate(distalVFilter.calculate(distalA), dtSeconds);
    }

    public double getLocalJointAngle() {
        return UtilityMath.normalizeAngleRadians(localPFilter.getCurrentOutput());
    }

    public double getLocalJointVelocity() {
        return localVFilter.getCurrentOutput();
    }

    public double getDistalJointAngle() {
        return UtilityMath.normalizeAngleRadians(distalPFilter.getCurrentOutput());
    }

    public double getDistalJointVelocity() {
        return distalVFilter.getCurrentOutput();
    }
}
