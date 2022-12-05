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
package org.chsrobotics.lib.drive;

import org.chsrobotics.lib.input.JoystickAxis;
import org.chsrobotics.lib.math.filters.RateLimiter;

public class CurvatureDrive implements DifferentialDriveMode {
    private final JoystickAxis linearAxis;
    private final JoystickAxis rotationalAxis;
    private final double turnModifier;
    private final double driveModifier;
    private final RateLimiter driveLimiter;
    private final RateLimiter turnLimiter;
    private final boolean invertReverseTurning;

    /**
     * Moves the robot in teleoperated mode similarly to {@link ArcadeDrive}, but with turning
     * speeds dependent on linear speeds.
     *
     * @param linearAxis The {@link JoystickAxis} to be used for linear movement.
     * @param rotationalAxis The {@link JoystickAxis} to be used for rotational movement.
     * @param driveModifier Adjusts the linear sensitivity.
     * @param turnModifier Adjusts the turn sensitivity.
     * @param driveLimiter The linear rate limit.
     * @param turnLimiter The rotational rate limit.
     * @param invertReverseTurning Whether turning in reverse should be inverted. This should be
     *     true when used in a MixedDrive.
     */
    public CurvatureDrive(
            JoystickAxis linearAxis,
            JoystickAxis rotationalAxis,
            double driveModifier,
            double turnModifier,
            double driveLimiter,
            double turnLimiter,
            boolean invertReverseTurning) {
        this.linearAxis = linearAxis;
        this.rotationalAxis = rotationalAxis;
        this.driveModifier = driveModifier;
        this.turnModifier = turnModifier;
        this.driveLimiter = new RateLimiter(driveLimiter);
        this.turnLimiter = new RateLimiter(turnLimiter);
        this.invertReverseTurning = invertReverseTurning;
    }

    /** {@inheritDoc} */
    @Override
    public DifferentialMove execute() {
        double rotationMultiplier =
                invertReverseTurning ? Math.abs(linearAxis.getValue()) : linearAxis.getValue();
        // rotation = (linear * rotational input)
        double rotation =
                turnLimiter.calculate((rotationalAxis.getValue() * rotationMultiplier))
                        * turnModifier;
        double linear = driveLimiter.calculate(linearAxis.getValue()) * driveModifier;

        double left = linear + rotation;
        double right = linear - rotation;

        return new DifferentialMove(left, right);
    }
}
