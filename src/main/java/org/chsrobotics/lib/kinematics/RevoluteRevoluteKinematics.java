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
package org.chsrobotics.lib.kinematics;

import org.chsrobotics.lib.util.Tuple2;

/**
 * Class for performing forward and inverse kinematic operations to determine the configuration of
 * an arm with 2 revolute (rotational) degrees of freedom.
 *
 * <p>Units of linear distance in this class are not specified, but must be consistent.
 */
public class RevoluteRevoluteKinematics {

    /** Data class holding information about the state of a double revolute-jointed arm. */
    public static class RRConfiguration {
        /**
         * Angle of joint A (the joint connected to the root), in radians, from (counterclockwise)
         * the horizontal (or the positive x-axis in non-vertically oriented setups).
         */
        public final double jointAAngle;

        /**
         * Angle of joint B (the joint between arm segments), in radians, from (counterclockwise)
         * parallel with joint A.
         */
        public final double jointBAngle;

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
         * @param jointAAngle Angle of joint A (the joint connected to the root), in radians, from
         *     (counterclockwise) the horizontal (or the positive x-axis in non-vertically oriented
         *     setups).
         * @param jointBAngle Angle of joint B (the joint between arm segments), in radians, from
         *     (counterclockwise) parallel with joint A.
         * @param endEffectorAngle Angle of the end effector, in radians, from (counterclockwise)
         *     the horizontal (or the positive x-axis in non-vertically oriented setups).
         * @param endEffectorX The x-position of the end effector in Cartesian space.
         * @param endEffectorY The y-position of the end effector in Cartesian space.
         */
        public RRConfiguration(
                double jointAAngle,
                double jointBAngle,
                double endEffectorAngle,
                double endEffectorX,
                double endEffectorY) {
            this.jointAAngle = jointAAngle;
            this.jointBAngle = jointBAngle;
            this.endEffectorAngle = endEffectorAngle;
            this.endEffectorX = endEffectorX;
            this.endEffectorY = endEffectorY;
        }

        @Override
        public String toString() {
            return ("RevoluteRevolute Configuration: jointAAngle: "
                    + jointAAngle
                    + ", jointBAngle: "
                    + jointBAngle
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

                return (rhs.jointAAngle == this.jointAAngle
                        && rhs.jointBAngle == this.jointBAngle
                        && rhs.endEffectorAngle == this.endEffectorAngle
                        && rhs.endEffectorX == this.endEffectorX
                        && rhs.endEffectorY == this.endEffectorY);
            } else return false;
        }
    }

    /** Length of arm segment A (the segment closest to the root). */
    public final double lengthA;

    /** Length of arm segment B (the segment closest to the end effector). */
    public final double lengthB;

    /**
     * Constructs a RevoluteRevoluteKinematics.
     *
     * <p>If either of the below are zero or less, this class's methods will return {@code null}.
     *
     * @param lengthA Length of arm segment A (the segment closest to the root).
     * @param lengthB Length of arm segment B (the segment closest to the end effector).
     */
    public RevoluteRevoluteKinematics(double lengthA, double lengthB) {
        this.lengthA = lengthA;
        this.lengthB = lengthB;
    }

    /**
     * Determines the position of the end effector in 2-dimensional space, given the angles of the
     * joints.
     *
     * @param jointAAngle Angle of joint A (the joint connected to the root), in radians, from
     *     (counterclockwise) the horizontal (or the positive x-axis in non-vertically oriented
     *     setups).
     * @param jointBAngle Angle of joint B (the joint between arm segments), in radians, from
     *     (counterclockwise) parallel with joint A.
     * @return A RRConfiguration describing the orientation.
     */
    public RRConfiguration forwardKinematics(double jointAAngle, double jointBAngle) {
        if (lengthA <= 0 || lengthB <= 0) return null;

        double endEffectorAngle = jointAAngle + jointBAngle;

        double x = (lengthA * Math.cos(jointAAngle)) + (lengthB * Math.cos(endEffectorAngle));
        double y = (lengthA * Math.sin(jointAAngle)) + (lengthB * Math.sin(endEffectorAngle));

        return new RRConfiguration(jointAAngle, jointBAngle, endEffectorAngle, x, y);
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
        if ((lengthA <= 0 || lengthB <= 0) || (endEffectorX == 0 && endEffectorY == 0))
            return Tuple2.of(null, null);

        double rootToEffectorVectorDirection = Math.atan2(endEffectorY, endEffectorX);

        RRConfiguration lefty;
        RRConfiguration righty;

        double cosAlpha =
                (Math.pow(endEffectorX, 2)
                                + Math.pow(endEffectorY, 2)
                                + Math.pow(lengthA, 2)
                                - Math.pow(lengthB, 2))
                        / (2
                                * lengthA
                                * Math.pow(
                                        Math.pow(endEffectorX, 2) + Math.pow(endEffectorY, 2),
                                        0.5));

        double cosBeta =
                (Math.pow(lengthA, 2)
                                + Math.pow(lengthB, 2)
                                - Math.pow(endEffectorX, 2)
                                - Math.pow(endEffectorY, 2))
                        / (2 * lengthA * lengthB);

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
