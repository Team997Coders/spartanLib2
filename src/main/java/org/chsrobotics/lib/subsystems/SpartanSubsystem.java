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
package org.chsrobotics.lib.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Interface extending WPILib's Subsystem, for the purpose of enabling resets of a Subsystem to a
 * known state.
 */
public interface SpartanSubsystem extends Subsystem {

    /**
     * Implementations of this method should reset the state of this subsystem to a known initial
     * state.
     */
    @NeedsOwnership
    public void resetState();
}
