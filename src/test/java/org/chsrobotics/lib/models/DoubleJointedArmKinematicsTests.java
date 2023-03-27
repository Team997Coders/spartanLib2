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
package org.chsrobotics.lib.models;

import static org.junit.Assert.assertEquals;

import org.chsrobotics.lib.math.UtilityMath;
import org.junit.Test;

public class DoubleJointedArmKinematicsTests {
    @Test
    public void DoubleJointedInverseKinematicsRejectsImpossible() {
        DoubleJointedArmKinematics kinematics = new DoubleJointedArmKinematics(1, 1);

        assertEquals(null, kinematics.inverseKinematics(3, 0).firstValue());
        assertEquals(null, kinematics.inverseKinematics(3, 0).secondValue());

        kinematics = new DoubleJointedArmKinematics(2, 1);

        assertEquals(null, kinematics.inverseKinematics(0, 0.5).firstValue());
        assertEquals(null, kinematics.inverseKinematics(0, 0.5).secondValue());
    }

    @Test
    public void DoubleJointedInverseKinematicsWorks() {
        DoubleJointedArmKinematics kinematics = new DoubleJointedArmKinematics(2, 1);

        assertEquals(
                -Math.acos(0.875),
                kinematics.inverseKinematics(2, 0).firstValue().localAngle,
                UtilityMath.defaultAbsoluteEpsilon);
        assertEquals(
                Math.acos(0.875),
                kinematics.inverseKinematics(2, 0).secondValue().localAngle,
                UtilityMath.defaultAbsoluteEpsilon);

        assertEquals(
                Math.PI - Math.acos(0.25),
                kinematics.inverseKinematics(2, 0).firstValue().distalAngle,
                UtilityMath.defaultAbsoluteEpsilon);

        assertEquals(
                Math.acos(0.25) - Math.PI,
                kinematics.inverseKinematics(2, 0).secondValue().distalAngle,
                UtilityMath.defaultAbsoluteEpsilon);
    }

    @Test
    public void DoubleJointedForwardKinematicsWorks() {
        DoubleJointedArmKinematics kinematics = new DoubleJointedArmKinematics(1, 1);

        assertEquals(
                2,
                kinematics.forwardKinematics(Math.PI / 2, 0).endEffectorY,
                UtilityMath.defaultAbsoluteEpsilon);

        assertEquals(
                0,
                kinematics.forwardKinematics(Math.PI / 2, 0).endEffectorX,
                UtilityMath.defaultAbsoluteEpsilon);

        assertEquals(
                Math.sqrt(2),
                kinematics.forwardKinematics(Math.PI / 4, 0).endEffectorX,
                UtilityMath.defaultAbsoluteEpsilon);

        assertEquals(
                Math.sqrt(2),
                kinematics.forwardKinematics(Math.PI / 4, 0).endEffectorY,
                UtilityMath.defaultAbsoluteEpsilon);

        assertEquals(
                (Math.sqrt(2) / 2) + 1,
                kinematics.forwardKinematics(Math.PI / 4, -Math.PI / 4).endEffectorX,
                UtilityMath.defaultAbsoluteEpsilon);

        assertEquals(
                Math.sqrt(2) / 2,
                kinematics.forwardKinematics(Math.PI / 4, -Math.PI / 4).endEffectorY,
                UtilityMath.defaultAbsoluteEpsilon);
    }
}
