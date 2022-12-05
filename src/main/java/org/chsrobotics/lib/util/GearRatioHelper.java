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
package org.chsrobotics.lib.util;

import java.security.InvalidParameterException;

/** Simple utility representing a gear ratio. */
public class GearRatioHelper {
    private final double inputRatio;
    private final double outputRatio;

    /**
     * Constructs a GearRatioHelper.
     *
     * <p>The parameters indicate the "input" and "output" sides of the mechanism. The input is
     * typically where the motor is.
     *
     * <p>The measurements of the parameters can be of many types that relate the motion of one side
     * to the other, as long as they're consistent. For example, radius, circumference, number of
     * gear teeth, angular distance per angular distance of the other, and more, are all valid
     * parameters.
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
     * Returns the gear ratio as output/input.
     *
     * @return The double ratio as output/input.
     */
    public double toDoubleRatioOutputToInput() {
        return outputRatio / inputRatio;
    }

    /**
     * Returns how far the "output" side of the mechanism will have travelled if the "input" has
     * travelled the indicated angular distance.
     *
     * @param inputSide The angular distance travelled by the *input* side.
     * @return The angular distance travelled by the *output* side.
     */
    public double outputFromInput(double inputSide) {
        return (inputSide * inputRatio) / outputRatio;
    }

    /**
     * Returns how far the "input" side of the mechanism will have travelled if the "output" has
     * travelled the indicated angular distance.
     *
     * @param outputSide The angular distance travelled by the *output* side.
     * @return The angular distance travelled by the *output* side.
     */
    public double inputFromOutput(double outputSide) {
        return (outputSide * outputRatio) / inputRatio;
    }
}
