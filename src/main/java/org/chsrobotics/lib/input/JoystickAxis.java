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
package org.chsrobotics.lib.input;

import java.util.function.Supplier;

/** Represents a hardware axis on an input device which has a value in [-1,1] (inclusive). */
public class JoystickAxis {
    private final Supplier<Double> valueLambda;
    private double lastValue = 0;
    private double deadband = 0;
    private final String name;
    private final boolean isReal;

    /**
     * Constructs a JoystickAxis.
     *
     * @param valueLambda Supplier of the value of the axis, in [-1,1] (inclusive).
     * @param name String identifier of the index of this axis in the driver station view.
     * @param isReal If this JoystickAxis is a representation of actual hardware.
     */
    protected JoystickAxis(Supplier<Double> valueLambda, String name, boolean isReal) {
        this.valueLambda = valueLambda;
        this.name = name;
        this.isReal = isReal;
    }

    /**
     * Returns the current value of the axis, with deadband applied.
     *
     * @return Value of the axis, in [-1,1] (inclusive).
     */
    public double getValue() {
        lastValue = (Math.abs(valueLambda.get()) > deadband) ? valueLambda.get() : 0;
        return lastValue;
    }

    /**
     * Adds a deadband to the axis such that axis absolute values below this will be returned as 0.
     *
     * @param magnitude The minimum absolute value required for the controller to not return 0.
     */
    public void addDeadband(double magnitude) {
        deadband = Math.abs(magnitude);
    }

    /**
     * Returns the approximated rate of change of the value of the axis.
     *
     * <p>Assumes that 20 milliseconds (default robot loop period) have passed between the last call
     * of {@code getValue} and this.
     *
     * @return The current approximate derivative of the axis' values with respect to time.
     */
    public double getRateOfChange() {
        return (valueLambda.get() - lastValue) / 0.02;
    }

    /**
     * Returns whether the value of the axis has increased since the last call of {@code getValue}.
     *
     * @return True if the axis is increasing in value.
     */
    public boolean isRising() {
        return (valueLambda.get() > lastValue);
    }

    /**
     * Returns whether the value of the axis has decreased since the last call of {@code getValue}.
     *
     * @return True if the axis is decreasing in value.
     */
    public boolean isFalling() {
        return (valueLambda.get() < lastValue);
    }

    /**
     * Returns whether the value of the axis has changed since the last call of {@code getValue}.
     *
     * @return True if the axis has changed in value.
     */
    public boolean hasChangedFromPreviousValue() {
        return (valueLambda.get() != lastValue);
    }

    /**
     * Whether this JoystickAxis is a representation of physical hardware.
     *
     * @return If this exists on an actual controller.
     */
    public boolean isReal() {
        return isReal;
    }

    @Override
    public String toString() {
        return ("JoystickAxis: " + name);
    }
}
