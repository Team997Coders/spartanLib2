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
package frc.team997.lib.Trajectory;

import edu.wpi.first.math.geometry.Pose2d;

/** Subclass of Pose2d that also holds a time value. */
public class TimestampedPose2d extends Pose2d {
    private long m_timeMillis;

    /**
     * Creates a TimestampedPose2d out of a specified pose and timestamp.
     *
     * @param pose The Pose2d of the robot.
     * @param timeMillis The value of the timestamp. Note that this does not include epoch.
     */
    public TimestampedPose2d(Pose2d pose, long timeMillis) {
        super(pose.getTranslation(), pose.getRotation());
        m_timeMillis = timeMillis;
    }

    /**
     * Creates a TimestampedPose2d out of a specified pose and System.currentTimeMillis(). Note that
     * this may be a different epoch from other instances of this class.
     *
     * @param pose The Pose2d of the robot.
     */
    public TimestampedPose2d(Pose2d pose) {
        this(pose, System.currentTimeMillis());
    }

    /**
     * Returns the timestamp. The epoch of the timestamp may not be uniform between instances of
     * this class.
     *
     * @return The value of the timestamp.
     */
    public long getTimeMillis() {
        return m_timeMillis;
    }
}
