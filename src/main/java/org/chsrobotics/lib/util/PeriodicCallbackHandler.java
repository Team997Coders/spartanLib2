/**
Copyright 2023 FRC Team 997

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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.DoubleConsumer;

/**
 * Class to facilitate callbacks to large numbers of periodic methods.
 *
 * <p>For this to function, {@code executeCallbacks()} needs to be called at your desired frequency,
 * most likely from your robot-wide {@code periodic()} method.
 *
 * <p>Don't use this as a replacement to {@code periodic()} in command-based Subsystems or {@code
 * execute()} in command-based Commands-- this is meant for coordinating tasks like differentiation
 * or updating logs.
 */
public class PeriodicCallbackHandler {
    private static class CallbackAndTimestamp {
        DoubleConsumer callback;
        double timestamp;

        CallbackAndTimestamp(DoubleConsumer callback, double timestamp) {
            this.callback = callback;
            this.timestamp = timestamp;
        }
    }

    private static final HashMap<Integer, CallbackAndTimestamp> timeCallbacks = new HashMap<>();

    private static final ArrayList<Runnable> voidCallbacks = new ArrayList<>();

    /**
     * Registers a double consumer for callbacks. The number passed during the callback cooresponds
     * to a time in seconds since last call of that method through this class (a dt).
     *
     * @param callback A double consumer (a void-returning method with a single double parameter).
     */
    public static void registerCallback(DoubleConsumer callback) {
        timeCallbacks.put(
                callback.hashCode(),
                new CallbackAndTimestamp(callback, System.currentTimeMillis()));
    }

    // TODO: update docs
    public static void registerCallback(Runnable callback) {
        voidCallbacks.add(callback);
    }

    /**
     * Deregisters a double consumer such that it will no longer be called by this class.
     *
     * @param callback A double consumer to no longer call back to.
     * @return Whether the double consumer could successfully be removed.
     */
    public static boolean deregisterCallback(DoubleConsumer callback) {
        return timeCallbacks.remove(callback.hashCode()) != null;
    }

    public static boolean deregisterCallback(Runnable callback) {
        return voidCallbacks.remove(callback);
    }

    /**
     * Executes all registered callbacks.
     *
     * <p>This method must be called regularly for many library functions to work. The author
     * suggests using your robot's {@code periodic()} method.
     */
    public static void executeCallbacks() {
        for (CallbackAndTimestamp entry : timeCallbacks.values()) {
            double currentTime = System.currentTimeMillis() / 1000;

            entry.callback.accept(currentTime - entry.timestamp);
            entry.timestamp = currentTime;
        }

        for (Runnable runnable : voidCallbacks) {
            runnable.run();
        }
    }
}
