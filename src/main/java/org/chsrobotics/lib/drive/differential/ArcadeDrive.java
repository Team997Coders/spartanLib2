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
package org.chsrobotics.lib.drive.differential;

import org.chsrobotics.lib.input.JoystickAxis;
import org.chsrobotics.lib.math.filters.RateLimiter;

/** Moves the robot in teleop using separate inputs for linear and rotational motion. */
public class ArcadeDrive implements DifferentialDriveMode {
    private final JoystickAxis linearAxis;
    private final JoystickAxis rotationalAxis;
    private final double driveModifier;
    private final double turnModifier;
    private final RateLimiter driveLimiter;
    private final RateLimiter turnLimiter;

    /**
     * Constructs an ArcadeDrive.
     *
     * @param linearAxis The {@link JoystickAxis} to be used for linear movement.
     * @param rotationalAxis The {@link JoystickAxis} to be used for rotational movement.
     * @param driveModifier A scalar to multiply the linear input by.
     * @param turnModifier A scalar to multiply the rotational input by.
     * @param driveLimiter The maximum rate of change of the linear input, in units of input /
     *     second. If equal to 0, no rate limiting will be applied.
     * @param turnLimiter The maximum rate of change of the rotational input, in units of input /
     *     second. If equal to 0, no rate limiting will be applied.
     */
    public ArcadeDrive(
            JoystickAxis linearAxis,
            JoystickAxis rotationalAxis,
            double driveModifier,
            double turnModifier,
            double driveLimiter,
            double turnLimiter) {
        this.linearAxis = linearAxis;
        this.rotationalAxis = rotationalAxis;
        this.driveModifier = driveModifier;
        this.turnModifier = turnModifier;
        this.driveLimiter = new RateLimiter(driveLimiter);
        this.turnLimiter = new RateLimiter(turnLimiter);
    }

    /** {@inheritDoc} */
    @Override
    public DifferentialDriveInput execute() {
        double linear = driveLimiter.calculate(linearAxis.getValue() * driveModifier);
        double rotation = turnLimiter.calculate(rotationalAxis.getValue() * turnModifier);

        double left = linear + rotation;
        double right = linear - rotation;

        return new DifferentialDriveInput(left, right);
    }
}
