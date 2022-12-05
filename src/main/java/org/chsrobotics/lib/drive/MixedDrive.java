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
package org.chsrobotics.lib.drive;

import java.util.Map;
import java.util.Optional;
import java.util.function.Predicate;

public class MixedDrive implements DifferentialDriveMode {
    final Map<DifferentialDriveMode, Double> driveModes;
    /**
     * Mixture of {@link ArcadeDrive} and {@link CurvatureDrive}. Proportions between the two set in
     * Constants.
     *
     * @param driveModes A map of DriveModes to mode weight
     */
    public MixedDrive(Map<DifferentialDriveMode, Double> driveModes) {
        if (driveModes.values().stream().anyMatch(Predicate.isEqual(null))) {
            throw new IllegalArgumentException("DriveMode proportions cannot be null");
        }
        if (!driveModes.values().stream().reduce(Double::sum).equals(Optional.of(1.0))) {
            throw new IllegalArgumentException("DriveMode proportions must add to 1");
        }
        this.driveModes = driveModes;
    }

    /** {@inheritDoc} */
    @Override
    public DifferentialMove execute() {
        DifferentialMove total = new DifferentialMove(0, 0);
        for (Map.Entry<DifferentialDriveMode, Double> entry : driveModes.entrySet()) {
            // get move for each drive mode, multiply it by that drive mode's proportion, and add it
            // to the total move
            total = total.add(entry.getKey().execute().multiply(entry.getValue()));
        }
        return total;
    }
}
