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

    // private final Map<RobotState, List<Runnable>> periodicCallbacks = new HashMap<>();

    // private final Map<Tuple2<RobotState>, List<Runnable>> stateTransitionCallbacks =
    //         new HashMap<>();

    private class WrappedTimedRobot extends TimedRobot {
        @Override
        public void teleopInit() {
            // stateTransitionRelay(currentState, RobotState.TELEOPERATED);

            stateTransition(currentState, RobotState.TELEOPERATED);
            currentState = RobotState.TELEOPERATED;
        }

        @Override
        public void autonomousInit() {
            // stateTransitionRelay(currentState, RobotState.AUTONOMOUS);

            stateTransition(currentState, RobotState.AUTONOMOUS);
            currentState = RobotState.AUTONOMOUS;
        }

        @Override
        public void testInit() {
            // stateTransitionRelay(currentState, RobotState.TEST);

            stateTransition(currentState, RobotState.TEST);
            currentState = RobotState.TEST;
        }

        @Override
        public void disabledInit() {
            if (DriverStation.isEStopped()) {
                // stateTransitionRelay(currentState, RobotState.ESTOPPED);

                stateTransition(currentState, RobotState.ESTOPPED);
                currentState = RobotState.ESTOPPED;
            } else {
                // stateTransitionRelay(currentState, RobotState.DISABLED);

                stateTransition(currentState, RobotState.DISABLED);
                currentState = RobotState.DISABLED;
            }
        }

        @Override
        public void robotPeriodic() {
            // periodicRelay(currentState);
            periodic(currentState);
        }
    }

    // private void stateTransitionRelay(RobotState from, RobotState to) {
    //     // have to be careful since null is an expected possible value here

    //     for (Entry<Tuple2<RobotState>, List<Runnable>> entry :
    //             stateTransitionCallbacks.entrySet()) {
    //         if (entry.getKey().firstValue() == null) { // catch from null
    //             if (entry.getKey().secondValue() != null) { // if to exists and applicable, run
    //                 if (entry.getKey().secondValue() == to) {
    //                     for (Runnable runnable : entry.getValue()) {
    //                         runnable.run();
    //                     }
    //                 }
    //             } else {
    //                 for (Runnable runnable : entry.getValue()) {
    //                     runnable.run(); // if to doesn't exist, run
    //                 }
    //             }
    //         } else if (entry.getKey().secondValue() == null) { // from exists but not to
    //             if (entry.getKey().firstValue() == from) { // if from applicable, run
    //                 for (Runnable runnable : entry.getValue()) {
    //                     runnable.run();
    //                 }
    //             }
    //         } else if (entry.getKey().firstValue() == from && entry.getKey().secondValue() == to)
    // {
    //             for (Runnable runnable : entry.getValue()) {
    //                 runnable.run(); // if both are applicable, run
    //             }
    //         }
    //     }

    //     stateTransition(from, to);
    // }

    // private void periodicRelay(RobotState state) {
    //     for (Entry<RobotState, List<Runnable>> entry : periodicCallbacks.entrySet()) {
    //         if (entry.getKey() != null) {
    //             if (entry.getKey() == state) {
    //                 for (Runnable runnable : entry.getValue()) {
    //                     runnable.run();
    //                 }
    //             }
    //         } else {
    //             for (Runnable runnable : entry.getValue()) {
    //                 runnable.run();
    //             }
    //         }
    //     }

    //     periodic(state);
    // }

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

    // /**
    //  * Registers a callback to run on execution of this class's {@code periodic()} method.
    //  *
    //  * <p>Note: methods called back like this will be executed before the {@code periodic()}
    // method
    //  * of this class.
    //  *
    //  * @param toRun The runnable to call.
    //  * @param state The state on which to call the runnable. If {@code null}, will call during
    // any
    //  *     state.
    //  */
    // public final void registerPeriodicCallback(Runnable toRun, RobotState state) {
    //     if (toRun != null) {
    //         if (periodicCallbacks.get(state) == null) {
    //             periodicCallbacks.put(state, new ArrayList<>());
    //         }

    //         periodicCallbacks.get(state).add(toRun);
    //     }
    // }

    // /**
    //  * Registers a callback to run on transition from one robot state to another.
    //  *
    //  * <p>Note: methods called back like this will be executed before the {@code
    // stateTransition()}
    //  * method of this class.
    //  *
    //  * @param toRun The runnable to call.
    //  * @param from The previous robot state to require. If {@code null}, will not be a constraint
    // to
    //  *     running the runnable.
    //  * @param to The current robot state to require. If {@code null}, will not be a constraint to
    //  *     running the runnable.
    //  */
    // public final void registerStateTransitionCallback(
    //         Runnable toRun, RobotState from, RobotState to) {
    //     if (toRun != null) {
    //         if (stateTransitionCallbacks.get(Tuple2.of(from, to)) == null) {
    //             stateTransitionCallbacks.put(Tuple2.of(from, to), new ArrayList<>());
    //         }

    //         stateTransitionCallbacks.get(Tuple2.of(from, to)).add(toRun);
    //     }
    // }

    /**
     * Returns whether the robot exists in a real or simulated environment.
     *
     * @return Whether or not the robot physically exists.
     */
    public final boolean isReal() {
        return WrappedTimedRobot.isReal();
    }

    /**
     * Returns the time, in each seconds, nominally taken by each robot loop cycle.
     *
     * @return The nominal length of each loop cycle.
     */
    public final double getPeriodSeconds() {
        return WrappedTimedRobot.kDefaultPeriod;
    }

    /**
     * Returns the current state of the robot.
     *
     * @return The current Driver Station control mode.
     */
    public final RobotState getRobotState() {
        return currentState;
    }
}
