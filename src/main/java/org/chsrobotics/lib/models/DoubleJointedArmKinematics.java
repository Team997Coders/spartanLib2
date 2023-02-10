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
package org.chsrobotics.lib.models;

import org.chsrobotics.lib.util.Tuple2;

/**
 * Class for performing forward and inverse kinematic operations to determine the configuration of
 * an arm with 2 revolute (rotational) degrees of freedom.
 *
 * <p>Units of linear distance in this class are not specified, but must be consistent.
 */
public class DoubleJointedArmKinematics {

    /** Data class holding information about the state of a double revolute-jointed arm. */
    public static class RRConfiguration {
        /**
         * Angle of the local joint (the joint connected to the root), in radians, from
         * (counterclockwise) the horizontal (or the positive x-axis in non-vertically oriented
         * setups).
         */
        public final double localAngle;

        /**
         * Angle of the distal joint (the joint between arm segments), in radians, from
         * (counterclockwise) parallel with joint A.
         */
        public final double distalAngle;

        /**
         * Angle of the end effector, in radians, from (counterclockwise) the horizontal (or the
         * positive x-axis in non-vertically oriented setups).
         */
        public final double endEffectorAngle;

        /** The x-position of the end effector in Cartesian space. */
        public final double endEffectorX;

        /** The y-position of the end effector in Cartesian space. */
        public final double endEffectorY;

        /**
         * Constructs an RRConfiguration.
         *
         * @param localAngle Angle of the local joint (the joint connected to the root), in radians,
         *     from (counterclockwise) the horizontal (or the positive x-axis in non-vertically
         *     oriented setups).
         * @param distalAngle Angle of the distal joint (the joint between arm segments), in
         *     radians, from (counterclockwise) parallel with the local joint.
         * @param endEffectorAngle Angle of the end effector, in radians, from (counterclockwise)
         *     the horizontal (or the positive x-axis in non-vertically oriented setups).
         * @param endEffectorX The x-position of the end effector in Cartesian space.
         * @param endEffectorY The y-position of the end effector in Cartesian space.
         */
        public RRConfiguration(
                double localAngle,
                double distalAngle,
                double endEffectorAngle,
                double endEffectorX,
                double endEffectorY) {
            this.localAngle = localAngle;
            this.distalAngle = distalAngle;
            this.endEffectorAngle = endEffectorAngle;
            this.endEffectorX = endEffectorX;
            this.endEffectorY = endEffectorY;
        }

        @Override
        public String toString() {
            return ("Double Jointed Arm Configuration: localAngle: "
                    + localAngle
                    + ", distalAngle: "
                    + distalAngle
                    + ", endEffectorAngle: "
                    + endEffectorAngle
                    + ", endEffectorX: "
                    + endEffectorX
                    + ", endEffectorY: "
                    + endEffectorY);
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof RRConfiguration) {
                RRConfiguration rhs = (RRConfiguration) other;

                return (rhs.localAngle == this.localAngle
                        && rhs.distalAngle == this.distalAngle
                        && rhs.endEffectorAngle == this.endEffectorAngle
                        && rhs.endEffectorX == this.endEffectorX
                        && rhs.endEffectorY == this.endEffectorY);
            } else return false;
        }
    }

    /** Length of the local segment (the segment closest to the root). */
    public final double localLength;

    /** Length of the distal segment (the segment closest to the end effector). */
    public final double distalLength;

    /**
     * Constructs a DoubleJointedArmKinematics.
     *
     * <p>If either of the below are zero or less, this class's methods will return {@code null}.
     *
     * @param localLength Length of the local segment (the segment closest to the root).
     * @param distalLength Length of the distal segment (the segment closest to the end effector).
     */
    public DoubleJointedArmKinematics(double localLength, double distalLength) {
        this.localLength = localLength;
        this.distalLength = distalLength;
    }

    /**
     * Determines the position of the end effector in 2-dimensional space, given the angles of the
     * joints.
     *
     * @param localAngle Angle of the local joint (the joint connected to the root), in radians,
     *     from (counterclockwise) the horizontal (or the positive x-axis in non-vertically oriented
     *     setups).
     * @param distalAngle Angle of the distal joint (the joint between arm segments), in radians,
     *     from (counterclockwise) parallel with the local joint.
     * @return A RRConfiguration describing the orientation.
     */
    public RRConfiguration forwardKinematics(double localAngle, double distalAngle) {
        if (localLength <= 0 || distalLength <= 0) return null;

        double endEffectorAngle = localAngle + distalAngle;

        double x =
                (localLength * Math.cos(localAngle)) + (distalLength * Math.cos(endEffectorAngle));
        double y =
                (localLength * Math.sin(localAngle)) + (distalLength * Math.sin(endEffectorAngle));

        return new RRConfiguration(localAngle, distalAngle, endEffectorAngle, x, y);
    }

    /**
     * Returns a tuple of 2 RRConfigurations with the joint angles required for the end effector to
     * reach a certain position in a Cartesian frame.
     *
     * <p>Two returned RRConfigurations are needed because there are 2 inverse kinematic solutions
     * to most possible cases. When only one solution exists, it is returned twice.
     *
     * <p>If the goal point is (0,0), there are either 0 or an infinite number of solutions,
     * depending on arm lengths, so a tuple of {@code null}s is returned.
     *
     * <p>This will also return {@code null}s if there are no solutions to reach the goal position.
     * This happens when {@code sqrt(x^2 + y^2) > lenA + lenB}, or {@code sqrt(x^2 + y^2) < |lenA -
     * lenB|}.
     *
     * <p>No guarantees are made about the actual feasibility of these configurations on hardware.
     *
     * @param endEffectorX The x-position of the end effector in Cartesian space.
     * @param endEffectorY The y-position of the end effector in Cartesian space.
     * @return A Tuple2 of up to 2 (possibly 0!) unique valid inverse kinematic solutions.
     */
    public Tuple2<RRConfiguration> inverseKinematics(double endEffectorX, double endEffectorY) {
        if ((localLength <= 0 || distalLength <= 0) || (endEffectorX == 0 && endEffectorY == 0))
            return Tuple2.of(null, null);

        double rootToEffectorVectorDirection = Math.atan2(endEffectorY, endEffectorX);

        RRConfiguration lefty;
        RRConfiguration righty;

        double cosAlpha =
                (Math.pow(endEffectorX, 2)
                                + Math.pow(endEffectorY, 2)
                                + Math.pow(localLength, 2)
                                - Math.pow(distalLength, 2))
                        / (2
                                * localLength
                                * Math.pow(
                                        Math.pow(endEffectorX, 2) + Math.pow(endEffectorY, 2),
                                        0.5));

        double cosBeta =
                (Math.pow(localLength, 2)
                                + Math.pow(distalLength, 2)
                                - Math.pow(endEffectorX, 2)
                                - Math.pow(endEffectorY, 2))
                        / (2 * localLength * distalLength);

        if (cosAlpha < -1 || cosAlpha > 1 || cosBeta < -1 || cosBeta > 1) {
            lefty = null;
            righty = null;
        } else {
            double thetaARighty = rootToEffectorVectorDirection - Math.acos(cosAlpha);
            double thetaBRighty = Math.PI - Math.acos(cosBeta);

            double thetaALefty = rootToEffectorVectorDirection + Math.acos(cosAlpha);
            double thetaBLefty = Math.acos(cosBeta) - Math.PI;

            righty =
                    new RRConfiguration(
                            thetaARighty,
                            thetaBRighty,
                            thetaARighty + thetaBRighty,
                            endEffectorX,
                            endEffectorY);

            lefty =
                    new RRConfiguration(
                            thetaALefty,
                            thetaBLefty,
                            thetaALefty + thetaBLefty,
                            endEffectorX,
                            endEffectorY);
        }

        return Tuple2.of(righty, lefty);
    }
}
