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
package org.chsrobotics.lib.input;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;
import java.util.Map;
import org.chsrobotics.lib.telemetry.HighLevelLogger;

/** Wrapper class for the Xbox controller used by FRC 997. */
public class XboxController {
    private final GenericHID controller;

    private double rightRumble = 0;
    private double leftRumble = 0;

    private final HashMap<Integer, Boolean> axisAllocations =
            new HashMap<>(
                    Map.ofEntries(
                            Map.entry(0, false),
                            Map.entry(1, false),
                            Map.entry(2, false),
                            Map.entry(3, false),
                            Map.entry(4, false),
                            Map.entry(5, false)));

    private final HashMap<Integer, Boolean> buttonAllocations =
            new HashMap<>(
                    Map.ofEntries(
                            Map.entry(1, false),
                            Map.entry(2, false),
                            Map.entry(3, false),
                            Map.entry(4, false),
                            Map.entry(5, false),
                            Map.entry(6, false),
                            Map.entry(7, false),
                            Map.entry(8, false),
                            Map.entry(9, false),
                            Map.entry(10, false)));

    private final EventLoop pollingLoop;

    /**
     * Constructs an XboxController connected to a specific port in the driver station application.
     *
     * @param pollingLoop The EventLoop used to poll button press actions.
     * @param port The port reported by the driver station for this controller.
     */
    public XboxController(EventLoop pollingLoop, int port) {
        controller = new GenericHID(port);
        this.pollingLoop = pollingLoop;
    }

    /**
     * Constructs an XboxController connected to a specific port in the driver station application,
     * using the default CommandScheduler event loop.
     *
     * @param port The port reported by the driver station for this controller.
     */
    public XboxController(int port) {
        this(CommandScheduler.getInstance().getDefaultButtonLoop(), port);
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
        return getJoystickAxis(1);
    }

    /**
     * Returns the JoystickAxis cooresponding to the left stick's horizontal axis. Throws a warning
     * if already allocated.
     *
     * @return The JoystickAxis at index 0.
     */
    public JoystickAxis leftStickHorizontalAxis() {
        return getJoystickAxis(0);
    }

    /**
     * Returns the JoystickAxis cooresponding to the right stick's vertical axis. Throws a warning
     * if already allocated.
     *
     * @return The JoystickAxis at index 5.
     */
    public JoystickAxis rightStickVerticalAxis() {
        return getJoystickAxis(5);
    }

    /**
     * Returns the JoystickAxis cooresponding to the right stick's horizontal axis. Throws a warning
     * if already allocated.
     *
     * @return The JoystickAxis at index 4.
     */
    public JoystickAxis rightStickHorizontalAxis() {
        return getJoystickAxis(4);
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
        return getJoystickAxis(2);
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
        return getJoystickAxis(3);
    }

    /**
     * Returns the JoystickButton cooresponding to the A button. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 1.
     */
    public JoystickButton AButton() {
        return getJoystickButton(1);
    }

    /**
     * Returns the JoystickButton cooresponding to the B button. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 2.
     */
    public JoystickButton BButton() {
        return getJoystickButton(2);
    }

    /**
     * Returns the JoystickButton cooresponding to the X button. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 3.
     */
    public JoystickButton XButton() {
        return getJoystickButton(3);
    }

    /**
     * Returns the JoystickButton cooresponding to the Y button. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 4.
     */
    public JoystickButton YButton() {
        return getJoystickButton(4);
    }

    /**
     * Returns the JoystickButton cooresponding to the left bumper. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 5.
     */
    public JoystickButton leftBumperButton() {
        return getJoystickButton(5);
    }

    /**
     * Returns the JoystickButton cooresponding to the right bumper. Throws a warning if already
     * allocated.
     *
     * @return The JoystickButton at index 6.
     */
    public JoystickButton rightBumperButton() {
        return getJoystickButton(6);
    }

    /**
     * Returns the JoystickButton cooresponding to the left stick button (pushing the stick into the
     * controller, should make a small click). Throws a warning if already allocated.
     *
     * @return The JoystickButton at index 9.
     */
    public JoystickButton leftStickButton() {
        return getJoystickButton(9);
    }

    /**
     * Returns the JoystickButton cooresponding to the right stick button (pushing the stick into
     * the controller, should make a small click). Throws a warning if already allocated.
     *
     * @return The JoystickButton at index 10.
     */
    public JoystickButton rightStickButton() {
        return getJoystickButton(10);
    }

    /**
     * Returns the JoystickButton cooresponding to the back button on the controller. Throws a
     * warning if already allocated.
     *
     * @return The JoystickButton at index 7.
     */
    public JoystickButton backButton() {
        return getJoystickButton(7);
    }

    /**
     * Returns the JoystickButton cooresponding to the start button on the controller. Throws a
     * warning if already allocated.
     *
     * @return The JoystickButton at index 8.
     */
    public JoystickButton startButton() {
        return getJoystickButton(8);
    }

    private JoystickButton getJoystickButton(int index) {
        if (buttonAllocations.get(index)) {
            HighLevelLogger.getInstance()
                    .logWarning("Joystick button at port " + index + " already allocated!");
        } else buttonAllocations.replace(index, true);

        return new JoystickButton(
                pollingLoop, () -> controller.getRawButton(index), Integer.toString(index), true);
    }

    private JoystickAxis getJoystickAxis(int index) {
        if (axisAllocations.get(index)) {
            HighLevelLogger.getInstance()
                    .logWarning("Joystick axis at port " + index + " already allocated!");
        } else {
            axisAllocations.replace(index, true);
        }
        return new JoystickAxis(() -> controller.getRawAxis(index), Integer.toString(index), true);
    }
}
