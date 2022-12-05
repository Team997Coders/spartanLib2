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

import edu.wpi.first.math.filter.SlewRateLimiter;
import org.chsrobotics.lib.input.JoystickAxis;

public class ArcadeDrive implements DifferentialDriveMode {
    private final JoystickAxis linearAxis;
    private final JoystickAxis rotationalAxis;
    private final double driveModifier;
    private final double turnModifier;
    private final SlewRateLimiter driveLimiter;
    private final SlewRateLimiter turnLimiter;

    /**
     * Moves the robot in teleoperated mode using separate inputs for linear and rotational
     * movement.
     *
     * @param linearAxis The {@link JoystickAxis} to be used for linear movement
     * @param rotationalAxis The {@link JoystickAxis} to be used for rotational movement
     * @param driveModifier Adjusts the linear sensitivity
     * @param turnModifier Adjusts the turn sensitivity
     * @param driveLimiter The linear rate limit
     * @param turnLimiter The rotational rate limit
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
        this.driveLimiter = new SlewRateLimiter(driveLimiter);
        this.turnLimiter = new SlewRateLimiter(turnLimiter);
    }

    /** {@inheritDoc} */
    @Override
    public DifferentialMove execute() {
        // left = linear - rotational
        double left =
                driveLimiter.calculate(linearAxis.getValue()) * driveModifier
                        + turnLimiter.calculate(rotationalAxis.getValue()) * turnModifier;
        // right = linear + rotational
        double right =
                driveLimiter.calculate(linearAxis.getValue()) * driveModifier
                        - turnLimiter.calculate(rotationalAxis.getValue()) * turnModifier;
        return new DifferentialMove(left, right);
    }
}
