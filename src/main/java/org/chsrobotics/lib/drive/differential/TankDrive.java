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
package org.chsrobotics.lib.drive.differential;

import java.util.function.Supplier;
import org.chsrobotics.lib.input.JoystickAxis;
import org.chsrobotics.lib.math.filters.RateLimiter;

/** Moves the robot in teleoperated mode using one input directly mapped to each wheel. */
public class TankDrive implements DifferentialDriveMode {
    private final Supplier<Double> leftAxis;
    private final Supplier<Double> rightAxis;
    private final Supplier<Double> driveModifier;
    private RateLimiter leftDriveLimiter;
    private RateLimiter rightDriveLimiter;

    /**
     * Constructs a TankDrive.
     *
     * @param leftAxis The {@link JoystickAxis} to be used for the left side.
     * @param rightAxis The {@link JoystickAxis} to be used for the right side.
     * @param driveModifier A scalar to multiply each input by.
     * @param driveLimiter The maximum rate of change for each input, in units of input / second. If
     *     equal to 0, no rate limiting will be applied.
     */
    public TankDrive(
            Supplier<Double> leftAxis,
            Supplier<Double> rightAxis,
            Supplier<Double> driveModifier,
            double driveLimiter) {
        this.leftAxis = leftAxis;
        this.rightAxis = rightAxis;
        this.driveModifier = driveModifier;
        this.leftDriveLimiter = new RateLimiter(driveLimiter);
        this.rightDriveLimiter = new RateLimiter(driveLimiter);
    }

    /**
     * Constructs a TankDrive with no rate limiting.
     *
     * @param leftAxis The {@link JoystickAxis} to be used for the left side.
     * @param rightAxis The {@link JoystickAxis} to be used for the right side.
     */
    public TankDrive(Supplier<Double> leftAxis, Supplier<Double> rightAxis) {
        this(leftAxis, rightAxis, () -> 1.0, 0);
    }

    /** {@inheritDoc} */
    @Override
    public DifferentialDrivetrainInput execute() {
        double left = leftAxis.get() * driveModifier.get();
        double right = rightAxis.get() * driveModifier.get();

        left = leftDriveLimiter.calculate(left);
        right = rightDriveLimiter.calculate(right);

        return new DifferentialDrivetrainInput(left, right).clamp(1);
    }
}
