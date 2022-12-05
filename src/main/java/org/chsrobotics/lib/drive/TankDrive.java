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

public class TankDrive implements DifferentialDriveMode {
    private final JoystickAxis leftAxis;
    private final JoystickAxis rightAxis;
    private final double driveModifier;
    private RateLimiter leftDriveLimiter;
    private RateLimiter rightDriveLimiter;

    /**
     * Moves the robot in teleoperated mode using one input directly mapped to each wheel.
     *
     * @param leftAxis The {@link JoystickAxis} to be used for the left wheel.
     * @param rightAxis The {@link JoystickAxis} to be used for the right wheel.
     * @param driveModifier Adjusts the linear sensitivity.
     * @param driveLimiter The linear rate limit.
     */
    public TankDrive(
            JoystickAxis leftAxis,
            JoystickAxis rightAxis,
            double driveModifier,
            double driveLimiter) {
        this.leftAxis = leftAxis;
        this.rightAxis = rightAxis;
        this.driveModifier = driveModifier;
        this.leftDriveLimiter = new RateLimiter(driveLimiter);
        this.rightDriveLimiter = new RateLimiter(driveLimiter);
    }

    /**
     * Moves the robot in teleoperated mode using one input directly mapped to each wheel.
     *
     * @param leftAxis The {@link JoystickAxis} to be used for the left wheel.
     * @param rightAxis The {@link JoystickAxis} to be used for the right wheel.
     */
    public TankDrive(JoystickAxis leftAxis, JoystickAxis rightAxis) {
        this.leftAxis = leftAxis;
        this.rightAxis = rightAxis;
        driveModifier = 1;
    }

    /** {@inheritDoc} */
    @Override
    public DifferentialMove execute() {
        double left = leftAxis.getValue() * driveModifier;
        double right = rightAxis.getValue() * driveModifier;
        if (leftDriveLimiter != null && rightDriveLimiter != null) {
            left = leftDriveLimiter.calculate(left);
            right = rightDriveLimiter.calculate(right);
        }
        return new DifferentialMove(left, right);
    }
}
