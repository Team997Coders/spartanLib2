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
package org.chsrobotics.lib.hardware.base.swerve;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import org.chsrobotics.lib.hardware.base.encoder.AbstractAbsoluteEncoder;
import org.chsrobotics.lib.hardware.base.encoder.AbstractEncoder;
import org.chsrobotics.lib.hardware.base.motorController.AbstractMotorController;
import org.chsrobotics.lib.telemetry.IntrinsicLoggable;
import org.chsrobotics.lib.util.GearRatioHelper;

// TODO docs
public class CoaxialSwerveModule implements IntrinsicLoggable {
    public static record CoaxialSwerveModuleInput(double driveInputVolts, double steerInputVolts) {}

    public static record CoaxialSwerveModuleState(
            double drivePosition, double driveVelocity, double steerAngle, double steerVelocity) {}

    private final AbstractMotorController driveMotor;
    private final AbstractMotorController steerMotor;

    private final AbstractEncoder driveFeedback;
    private final AbstractAbsoluteEncoder steerFeedback;

    public CoaxialSwerveModule(
            AbstractMotorController driveMotor,
            AbstractMotorController steerMotor,
            AbstractEncoder driveFeedback,
            AbstractAbsoluteEncoder steerFeedback,
            GearRatioHelper driveGearRatio,
            double wheelRadiusMeters) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        this.driveFeedback =
                driveFeedback.withGearRatio(
                        driveGearRatio.addStage(new GearRatioHelper(1, wheelRadiusMeters)));
        // units already in radians so no need for 2pi
        this.steerFeedback = steerFeedback;
    }

    public AbstractMotorController getDriveMotor() {
        return driveMotor;
    }

    public AbstractMotorController getSteerMotor() {
        return steerMotor;
    }

    /**
     * not same as what was passed in
     *
     * @return
     */
    public AbstractEncoder getDriveFeedback() {
        return driveFeedback;
    }

    public AbstractAbsoluteEncoder getSteerFeedback() {
        return steerFeedback;
    }

    public void setInput(CoaxialSwerveModuleInput input) {
        driveMotor.setVoltage(input.driveInputVolts);

        steerMotor.setVoltage(input.steerInputVolts);
    }

    public CoaxialSwerveModuleState getState() {
        return new CoaxialSwerveModuleState(
                driveFeedback.getConvertedPosition(),
                driveFeedback.getConvertedVelocity(),
                steerFeedback.getConvertedPosition(),
                steerFeedback.getConvertedVelocity());
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        driveMotor.autoGenerateLogs(
                log, name + "/driveMotor", subdirName, publishToNT, recordInLog);

        steerMotor.autoGenerateLogs(
                log, name + "/steerMotor", subdirName, publishToNT, recordInLog);

        driveFeedback.autoGenerateLogs(
                log, name + "/driveEncoder", subdirName, publishToNT, recordInLog);

        steerFeedback.autoGenerateLogs(
                log, name + "/steerEncoder", subdirName, publishToNT, recordInLog);
    }

    public static enum Mk4Gearing {
        L1(new GearRatioHelper(1, 8.14)),
        L2(new GearRatioHelper(1, 6.75)),
        L3(new GearRatioHelper(1, 6.12)),
        L4(new GearRatioHelper(1, 5.14));

        private final GearRatioHelper gearing;

        private Mk4Gearing(GearRatioHelper gearing) {
            this.gearing = gearing;
        }
    }

    public static CoaxialSwerveModule getMk4(
            AbstractMotorController driveMotor,
            AbstractMotorController steerMotor,
            AbstractEncoder driveFeedback,
            AbstractAbsoluteEncoder steerFeedback,
            Mk4Gearing driveGearing) {
        return new CoaxialSwerveModule(
                driveMotor,
                steerMotor,
                driveFeedback,
                steerFeedback,
                driveGearing.gearing,
                Units.inchesToMeters(4));
    }
}
