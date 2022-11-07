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
package frc.team997.lib.util;

import java.security.InvalidParameterException;

/** Simple utility representing a gear ratio. */
public class GearRatioHelper {
    private final double inputRatio;
    private final double outputRatio;

    /**
     * Constructs a GearRatioHelper.
     *
     * <p>The parameters indicate the "input" and "output" sides of the mechanism. Should be
     * measured in distance of travel.
     *
     * <p>A higher output ratio than input ratio is a reduction (lower speed, higher torque).
     *
     * <p>A higher input ratio than output ratio is a step-up (higher speed, lower torque).
     *
     * @param inputRatio The measurement of the "input" side (usually where the motor is)
     * @param outputRatio The measurement of the "output" side (usually on the other side of a
     *     gearbox or series of pulleys from the motor)
     * @throws InvalidParameterException If the input or output ratio(s) are less than zero.
     */
    public GearRatioHelper(double inputRatio, double outputRatio) throws InvalidParameterException {
        if (inputRatio <= 0 || outputRatio <= 0) {
            throw new InvalidParameterException("Input/output ratios must be greater than 0!");
        }
        this.inputRatio = inputRatio;
        this.outputRatio = outputRatio;
    }

    /**
     * Creates a gear ratio helper from a double ratio of input to output.
     *
     * @param ratioInputToOutput The ratio as input/output.
     */
    public GearRatioHelper(double ratioInputToOutput) {
        this(ratioInputToOutput, 1);
    }

    /**
     * Returns the gear ratio as input/output.
     *
     * @return The double ratio as input/output.
     */
    public double toDoubleRatioInputToOutput() {
        return inputRatio / outputRatio;
    }

    /**
     * Returns how far the "output" side of the mechanism will have travelled if the "input" has
     * travelled the indicated distance.
     *
     * @param inputSide The distance travelled by the *input* side.
     * @return The distance travelled by the *output* side.
     */
    public double outputFromInput(double inputSide) {
        return (inputSide * inputRatio) / outputRatio;
    }

    /**
     * Returns how far the "input" side of the mechanism will have travelled if the "output" has
     * travelled the indicated distance.
     *
     * @param outputSide The distance travelled by the *output* side.
     * @return The distance travelled by the *output* side.
     */
    public double inputFromOutput(double outputSide) {
        return (outputSide * outputRatio) / inputRatio;
    }
}
