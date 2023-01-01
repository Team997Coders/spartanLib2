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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Wrapper around TimedRobot which replaces {@code xyzInit()} and {@code xyzPeriodic()} methods with
 * single unified state transition and periodic methods, with robot state given as parameters.
 */
public class SRobot {

    /** Enum of possible DriverStation robot control states. */
    public enum RobotState {
        TELEOPERATED,
        AUTONOMOUS,
        TEST,
        DISABLED,
        ESTOPPED,
        NONE
    }

    private RobotState currentState = RobotState.NONE;

    private class WrappedTimedRobot extends TimedRobot {
        @Override
        public void teleopInit() {
            stateTransition(currentState, RobotState.TELEOPERATED);
            currentState = RobotState.TELEOPERATED;
        }

        @Override
        public void autonomousInit() {
            stateTransition(currentState, RobotState.AUTONOMOUS);
            currentState = RobotState.AUTONOMOUS;
        }

        @Override
        public void testInit() {
            stateTransition(currentState, RobotState.TEST);
            currentState = RobotState.TEST;
        }

        @Override
        public void disabledInit() {
            if (DriverStation.isEStopped()) {
                stateTransition(currentState, RobotState.ESTOPPED);
                currentState = RobotState.ESTOPPED;
            } else {
                stateTransition(currentState, RobotState.DISABLED);
                currentState = RobotState.DISABLED;
            }
        }

        @Override
        public void robotPeriodic() {
            periodic(currentState);

            if (isSimulation()) simPeriodic(currentState);
        }
    }

    /** Call this method to start robot code execution. */
    public final void start() {
        RobotBase.startRobot(WrappedTimedRobot::new);
    }

    /**
     * Override this method and put your own logic here!
     *
     * <p>Method called whenever the robot base goes from one driver station control mode to
     * another.
     *
     * @param from The previous RobotState.
     * @param to The new RobotState.
     */
    public void stateTransition(RobotState from, RobotState to) {}

    /**
     * Override this method and put your own logic here!
     *
     * <p>Method called every loop of the robot base.
     *
     * @param state The current RobotState.
     */
    public void periodic(RobotState state) {}

    /**
     * Override this method and put your own logic here!
     *
     * <p>Method called every loop of the robot base while in simulation.
     *
     * @param state The current RobotState.
     */
    public void simPeriodic(RobotState state) {}
}
