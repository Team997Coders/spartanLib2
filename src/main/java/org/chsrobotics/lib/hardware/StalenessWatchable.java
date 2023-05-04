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
package org.chsrobotics.lib.hardware;

/** Interface for hardware which can self-check if it might be disconnected. */
public interface StalenessWatchable {
    public final int defaultStalenessThresholdCycles = 10;

    /**
     * Returns whether this piece of hardware is potentially stale (not communicating).
     *
     * <p>Some implementations watch for communications, others monitor noisy data that is very
     * unlikely to remain the same for many cycles.
     *
     * @return Whether this hardware might be disconnected.
     */
    public boolean isStale();
}
