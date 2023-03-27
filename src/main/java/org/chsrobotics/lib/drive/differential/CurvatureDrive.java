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

/**
 * Moves the robot in teleoperated mode similarly to {@link ArcadeDrive}, but with turning speeds
 * dependent on linear speeds.
 */
public class CurvatureDrive implements DifferentialDriveMode {
    private final Supplier<Double> linearAxis;
    private final Supplier<Double> rotationalAxis;
    private final Supplier<Double> turnModifier;
    private final Supplier<Double> driveModifier;
    private final RateLimiter driveLimiter;
    private final RateLimiter turnLimiter;
    private final boolean invertReverseTurning;

    /**
     * Constructs a CurvatureDrive.
     *
     * @param linearAxis The {@link JoystickAxis} to be used for linear movement.
     * @param rotationalAxis The {@link JoystickAxis} to be used for rotational movement.
     * @param driveModifier A scalar to multiply the linear input by.
     * @param turnModifier A scalar to multiply the rotational input by.
     * @param driveLimiter The maximum rate of change of the linear input, in units of input /
     *     second. If equal to 0, no rate limiting will be applied.
     * @param turnLimiter The maximum rate of change of the rotational input, in units of input /
     *     second. If equal to 0, no rate limiting will be applied.
     * @param invertReverseTurning Whether turning in reverse should be inverted.
     */
    public CurvatureDrive(
            Supplier<Double> linearAxis,
            Supplier<Double> rotationalAxis,
            Supplier<Double> driveModifier,
            Supplier<Double> turnModifier,
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
    public DifferentialDrivetrainInput execute() {
        double rotationMultiplier =
                invertReverseTurning ? Math.abs(linearAxis.get()) : linearAxis.get();
        // rotation = (linear * rotational input)
        double rotation =
                turnLimiter.calculate(
                        (rotationalAxis.get() * rotationMultiplier * turnModifier.get()));
        double linear = driveLimiter.calculate(linearAxis.get() * driveModifier.get());

        double left = linear + rotation;
        double right = linear - rotation;

        return new DifferentialDrivetrainInput(left, right).clamp(1);
    }
}
