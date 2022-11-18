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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import java.util.List;
import org.chsrobotics.lib.telemetry.HighLevelLogger;

/** Wrapper class for the Xbox controller used by FRC 997. */
public class XboxController {
    private final GenericHID controller;
    private double rightRumble = 0;
    private double leftRumble = 0;
    private final List<Boolean> axisAllocations = List.of(false, false, false, false, false, false);
    private final List<Boolean> buttonAllocations =
            List.of(false, false, false, false, false, false, false, false, false, false);

    /**
     * Constructs an XboxController connected to a specific port in the driver station application.
     *
     * @param port The port reported by the driver station for this controller.
     */
    public XboxController(int port) {
        controller = new GenericHID(port);
    }

    /**
     * Sets the rumble of the right side of the controller, for driver feedback.
     *
     * @param value The rumble value in [0,1] (inclusive).
     */
    public void setRightRumble(double value) {
        controller.setRumble(RumbleType.kRightRumble, value);
        rightRumble = value;
    }

    /**
     * Sets the rumble of the left side of the controller, for driver feedback.
     *
     * @param value The rumble value in [0,1] (inclusive).
     */
    public void setLeftRumble(double value) {
        controller.setRumble(RumbleType.kLeftRumble, value);
        leftRumble = value;
    }

    /**
     * Returns the currently applied right-side rumble value.
     *
     * @return The current rumble value, in [0,1] (inclusive).
     */
    public double getRightRumble() {
        return rightRumble;
    }

    /**
     * Returns the currently applied left-side rumble value.
     *
     * @return The current rumble value, in [0,1] (inclusive).
     */
    public double getLeftRumble() {
        return leftRumble;
    }

    /**
     * Returns the JoystickAxis cooresponding to the left stick's vertical axis. Throws a warning if
     * already allocated.
     *
     * @return The JoystickAxis at index 1.
     */
    public JoystickAxis leftStickVerticalAxis() {
        if (!axisAllocations.get(1)) {
            HighLevelLogger.logWarning("JoystickAxis already allocated!");
        } else {
            axisAllocations.set(1, true);
        }
        return new JoystickAxis(() -> controller.getRawAxis(1), "1", true);
    }

    /**
     * Returns the JoystickAxis cooresponding to the left stick's horizontal axis. Throws a warning
     * if already allocated.
     *
     * @return The JoystickAxis at index 0.
     */
    public JoystickAxis leftStickHorizontalAxis() {
        if (!axisAllocations.get(0)) {
            HighLevelLogger.logWarning("JoystickAxis already allocated!");
        } else {
            axisAllocations.set(0, true);
        }
        return new JoystickAxis(() -> controller.getRawAxis(0), "0", true);
    }

    /**
     * Returns the JoystickAxis cooresponding to the right stick's vertical axis. Throws a warning
     * if already allocated.
     *
     * @return The JoystickAxis at index 5.
     */
    public JoystickAxis rightStickVerticalAxis() {
        if (!axisAllocations.get(5)) {
            HighLevelLogger.logWarning("JoystickAxis already allocated!");
        } else {
            axisAllocations.set(5, true);
        }
        return new JoystickAxis(() -> controller.getRawAxis(5), "5", true);
    }

    /**
     * Returns the JoystickAxis cooresponding to the right stick's horizontal axis. Throws a warning
     * if already allocated.
     *
     * @return The JoystickAxis at index 4.
     */
    public JoystickAxis rightStickHorizontalAxis() {
        if (!axisAllocations.get(4)) {
            HighLevelLogger.logWarning("JoystickAxis already allocated!");
        } else {
            axisAllocations.set(4, true);
        }
        return new JoystickAxis(() -> controller.getRawAxis(4), "4", true);
    }

    /**
     * Returns the JoystickAxis cooresponding to the left trigger. Throws a warning if already
     * allocated.
     *
     * <p>The triggers don't follow the conventions of the other axis regarding value for the
     * mechanical default position.
     *
     * @return The JoystickAxis at index 2.
     */
    public JoystickAxis leftTriggerAxis() {
        if (!axisAllocations.get(2)) {
            HighLevelLogger.logWarning("JoystickAxis already allocated!");
        } else {
            axisAllocations.set(2, true);
        }
        return new JoystickAxis(() -> controller.getRawAxis(2), "2", true);
    }

    /**
     * Returns the JoystickAxis cooresponding to the right trigger. Throws a warning if already
     * allocated.
     *
     * <p>The triggers don't follow the conventions of the other axis regarding value for the
     * mechanical default position.
     *
     * @return The JoystickAxis at index 3.
     */
    public JoystickAxis rightTriggerAxis() {
        if (!axisAllocations.get(3)) {
            HighLevelLogger.logWarning("JoystickAxis already allocated!");
        } else {
            axisAllocations.set(3, true);
        }
        return new JoystickAxis(() -> controller.getRawAxis(3), "3", true);
    }

    /**
     * Returns the JoystickButton cooresponding to the A button. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 1.
     */
    public JoystickButton AButton() {
        if (!buttonAllocations.get(0)) {
            HighLevelLogger.logWarning("JoystickButton already allocated!");
        } else {
            buttonAllocations.set(0, true);
        }
        return new JoystickButton(() -> controller.getRawButton(1), "1", true);
    }

    /**
     * Returns the JoystickButton cooresponding to the B button. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 2.
     */
    public JoystickButton BButton() {
        if (!buttonAllocations.get(1)) {
            HighLevelLogger.logWarning("JoystickButton already allocated!");
        } else {
            buttonAllocations.set(1, true);
        }
        return new JoystickButton(() -> controller.getRawButton(2), "2", true);
    }

    /**
     * Returns the JoystickButton cooresponding to the X button. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 3.
     */
    public JoystickButton XButton() {
        if (!buttonAllocations.get(2)) {
            HighLevelLogger.logWarning("JoystickButton already allocated!");
        } else {
            buttonAllocations.set(2, true);
        }
        return new JoystickButton(() -> controller.getRawButton(3), "3", true);
    }

    /**
     * Returns the JoystickButton cooresponding to the Y button. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 4.
     */
    public JoystickButton YButton() {
        if (!buttonAllocations.get(3)) {
            HighLevelLogger.logWarning("JoystickButton already allocated!");
        } else {
            buttonAllocations.set(3, true);
        }
        return new JoystickButton(() -> controller.getRawButton(4), "4", true);
    }

    /**
     * Returns the JoystickButton cooresponding to the left bumper. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 5.
     */
    public JoystickButton leftBumperButton() {
        if (!buttonAllocations.get(4)) {
            HighLevelLogger.logWarning("JoystickButton already allocated!");
        } else {
            buttonAllocations.set(4, true);
        }
        return new JoystickButton(() -> controller.getRawButton(5), "5", true);
    }

    /**
     * Returns the JoystickButton cooresponding to the right bumper. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 6.
     */
    public JoystickButton rightBumperButton() {
        if (!buttonAllocations.get(5)) {
            HighLevelLogger.logWarning("JoystickButton already allocated!");
        } else {
            buttonAllocations.set(5, true);
        }
        return new JoystickButton(() -> controller.getRawButton(6), "6", true);
    }

    /**
     * Returns the JoystickButton cooresponding to the left stick button (pushing the stick into the
     * controller, should make a small click). Throws a warning if already allocated.
     *
     * @return The JoystickButton at index 9.
     */
    public JoystickButton leftStickButton() {
        if (!buttonAllocations.get(8)) {
            HighLevelLogger.logWarning("JoystickButton already allocated!");
        } else {
            buttonAllocations.set(8, true);
        }
        return new JoystickButton(() -> controller.getRawButton(9), "9", true);
    }

    /**
     * Returns the JoystickButton cooresponding to the right stick button (pushing the stick into
     * the controller, should make a small click). Throws a warning if already allocated.
     *
     * @return The JoystickButton at index 10.
     */
    public JoystickButton rightStickButton() {
        if (!buttonAllocations.get(9)) {
            HighLevelLogger.logWarning("JoystickButton already allocated!");
        } else {
            buttonAllocations.set(9, true);
        }
        return new JoystickButton(() -> controller.getRawButton(10), "10", true);
    }

    /**
     * Returns the JoystickButton cooresponding to the back button on the controller. Throws a
     * warning if already allocated.
     *
     * @return The JoystickButton at index 7.
     */
    public JoystickButton backButton() {
        if (!buttonAllocations.get(6)) {
            HighLevelLogger.logWarning("JoystickButton already allocated!");
        } else {
            buttonAllocations.set(6, true);
        }
        return new JoystickButton(() -> controller.getRawButton(7), "7", true);
    }

    /**
     * Returns the JoystickButton cooresponding to the start button on the controller. Throws a
     * warning if already allocated.
     *
     * @return The JoystickButton at index 8.
     */
    public JoystickButton startButton() {
        if (!buttonAllocations.get(7)) {
            HighLevelLogger.logWarning("JoystickButton already allocated!");
        } else {
            buttonAllocations.set(7, true);
        }
        return new JoystickButton(() -> controller.getRawButton(8), "8", true);
    }
}
