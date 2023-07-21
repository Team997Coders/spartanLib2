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

import edu.wpi.first.wpilibj.Timer;

/**
 * Class to simplify a common robot code action: getting a time elapsed between two points.
 *
 * <p>This class relies on WPI's {@code Timer.getFPGATimestamp()} method for its source of time.
 */
public class DeltaTimeUtil {
    private double lastTimestampSeconds;

    /**
     * Constructs an instance of DeltaTimeUtil.
     *
     * <p>The comparison base is initially set to the time as this is constructed.
     */
    public DeltaTimeUtil() {
        lastTimestampSeconds = Timer.getFPGATimestamp();
    }

    /**
     * Returns the time elapsed between now and the comparison base. The comparison base is set to
     * now after computation of the elapsed time.
     *
     * <p>Time determined by WPI's {@code Timer.getFPGATimestamp()} method.
     *
     * @return Time, in seconds, elapsed between now and the comparison base.
     */
    public double getTimeSecondsSinceLastCall() {
        double currentTime = Timer.getFPGATimestamp();

        double dt = currentTime - lastTimestampSeconds;

        lastTimestampSeconds = currentTime;

        return dt;
    }
}
