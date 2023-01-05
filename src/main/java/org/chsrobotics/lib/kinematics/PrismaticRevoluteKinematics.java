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
 * an arm with one prismatic (linear) joint and one revolute (rotational) joint, with the prismatic
 * joint vertically (or coplanar to y-axis) connected to the root.
 *
 * <p>Units of linear distance in this class are not specified, but must be consistent.
 */
public class PrismaticRevoluteKinematics {

    /** Data class holding information about the state of a prismatic-revolute arm. */
    public static class PRConfiguration {
        /** Signed extension of the prismatic joint vertically (+y) from the root. */
        public final double extension;

        /** Angle of the rotational joint from the horizontal (+x), counterclockwise in radians. */
        public final double rotation;

        /** Cartesian x-coordinate of the end effector. */
        public final double endEffectorX;

        /** Cartesian y-coordinate of the end effector. */
        public final double endEffectorY;

        /**
         * Constructs a PRConfiguration.
         *
         * @param extension Signed extension of the prismatic joint vertically (+y) from the root.
         * @param rotation Angle of the rotational joint from the horizontal (+x), counterclockwise
         *     in radians.
         * @param endEffectorX Cartesian x-coordinate of the end effector.
         * @param endEffectorY Cartesian y-coordinate of the end effector.
         */
        public PRConfiguration(
                double extension, double rotation, double endEffectorX, double endEffectorY) {
            this.extension = extension;
            this.rotation = rotation;

            this.endEffectorX = endEffectorX;
            this.endEffectorY = endEffectorY;
        }

        @Override
        public String toString() {
            return ("PrismaticRevolute Configuration: extension: "
                    + extension
                    + ", rotation: "
                    + rotation
                    + ", endEffectorX: "
                    + endEffectorX
                    + ", endEffectorY: "
                    + endEffectorY);
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof PRConfiguration) {
                PRConfiguration rhs = (PRConfiguration) other;

                return (this.extension == rhs.extension
                        && this.rotation == rhs.rotation
                        && this.endEffectorX == rhs.endEffectorX
                        && this.endEffectorY == rhs.endEffectorY);
            } else return false;
        }
    }

    /**
     * Length of the arm segment (the segment attached between the rotational joint and the end
     * effector).
     */
    public final double armLength;

    /**
     * Constructs a PrismaticRevoluteKinematics.
     *
     * <p>If this is zero or less, this class's methods will return {@code null}.
     *
     * @param armLength Length of the arm segment (the segment attached between the rotational joint
     *     and the end effector).
     */
    public PrismaticRevoluteKinematics(double armLength) {
        this.armLength = armLength;
    }

    /**
     * Determines the position of the end effector in 2-dimensional space, given the angle and
     * extension of the joints.
     *
     * @param extension Signed extension of the prismatic joint vertically (+y) from the root.
     * @param rotation Angle of the rotational joint from the horizontal (+x), counterclockwise in
     *     radians.
     * @return A PRConfiguration specifying the state of the prismatic-revolute arm.
     */
    public PRConfiguration forwardKinematics(double extension, double rotation) {
        return new PRConfiguration(
                extension,
                rotation,
                extension + (armLength * Math.cos(rotation)),
                (armLength * Math.sin(rotation)));
    }

    /**
     * Returns a tuple of 2 PRConfigurations with the joint angle and joint extension required for
     * the end effector to reach a certain position in a Cartesian frame.
     *
     * <p>Two returned PRConfigurations are needed because there are often 2 inverse kinematic
     * solutions, depending on the specific goal x and y. 1-solution situations can occur, in which
     * case that solution is returned twice.
     *
     * <p>This will return {@code null}s if there are no solutions to reach the goal position. This
     * happens when {@code |x| > armLength}.
     *
     * <p>No guarantees are made about the actual feasibility of these configurations on hardware.
     *
     * @param endEffectorX The x-position of the end effector in Cartesian space.
     * @param endEffectorY The y-position of the end effector in Cartesian space.
     * @return A Tuple2 of 2 (or 0!) unique valid inverse kinematic solutions.
     */
    public Tuple2<PRConfiguration> inverseKinematics(double endEffectorX, double endEffectorY) {
        PRConfiguration upper;
        PRConfiguration lower;

        if (Math.abs(endEffectorX) > armLength) {
            upper = null;
            lower = null;
        } else {
            double upperAngle = Math.acos(endEffectorX / armLength);
            double lowerAngle = -upperAngle;

            double upperYOffset = armLength * Math.sin(upperAngle);
            double lowerYOffset = armLength * Math.sin(lowerAngle);

            upper =
                    new PRConfiguration(
                            endEffectorY + upperYOffset, upperAngle, endEffectorX, endEffectorY);

            lower =
                    new PRConfiguration(
                            endEffectorY + lowerYOffset, lowerAngle, endEffectorX, endEffectorY);
        }

        return Tuple2.of(upper, lower);
    }
}
