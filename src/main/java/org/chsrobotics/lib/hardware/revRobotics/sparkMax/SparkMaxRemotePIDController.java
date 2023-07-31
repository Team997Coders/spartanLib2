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
package org.chsrobotics.lib.hardware.revRobotics.sparkMax;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import org.chsrobotics.lib.hardware.base.motorController.RemotePIDController;
import org.chsrobotics.lib.math.UtilityMath;

// TODO docs

public class SparkMaxRemotePIDController extends RemotePIDController {

    public static record SparkMaxRemoteControllerConfig(
            double initialSetpoint,
            RemoteControlType controlType,
            double initialFeedforwardVolts,
            double initialkP,
            double initialkI,
            double initialkD,
            double initialMaxControlEffort,
            double initialMaxIntegralControlEffort) {
        public SparkMaxRemoteControllerConfig(
                double initialSetpoint,
                RemoteControlType controlType,
                double initialFeedforwardVolts,
                double initialkP,
                double initialkI,
                double initialkD) {
            this(
                    initialSetpoint,
                    controlType,
                    initialFeedforwardVolts,
                    initialkP,
                    initialkI,
                    initialkD,
                    0,
                    0);
        }

        public SparkMaxRemoteControllerConfig(
                double initialSetpoint, RemoteControlType controlType) {
            this(initialSetpoint, controlType, 0, 0, 0, 0);
        }
    }

    private final SparkMaxRemoteControllerConfig config;

    private final SparkMaxPIDController controller;

    private double localFeedforwardVolts = 0;

    private double localkP = 0;
    private double localkI = 0;
    private double localkD = 0;

    private double localMaxU = 0;
    private double localMaxIU = 0;

    private double flushedFeedforward = 0;

    private double flushedkP = 0;
    private double flushedkI = 0;
    private double flushedkD = 0;

    private double flushedMaxU = 0;
    private double flushedMaxIU = 0;

    private double localSetpoint = 0;

    protected SparkMaxRemotePIDController(
            SparkMaxRemoteControllerConfig config, SparkMaxPIDController controller) {
        this.config = config;

        localFeedforwardVolts = config.initialFeedforwardVolts;

        localkP = config.initialkP;
        localkI = config.initialkI;
        localkD = config.initialkD;

        localMaxU = config.initialMaxControlEffort;
        localMaxIU = config.initialMaxIntegralControlEffort;

        this.controller = controller;

        setSetpoint(config.initialSetpoint);
    }

    @Override
    public RemoteControlType getControlType() {
        return config.controlType;
    }

    @Override
    public void setFeedforwardVolts(double feedforwardVolts) {
        localFeedforwardVolts = feedforwardVolts;
    }

    @Override
    public double getFeedforwardVolts() {
        return flushedFeedforward;
    }

    @Override
    public double getkP() {
        return flushedkP;
    }

    @Override
    public void setkP(double kP) {
        localkP = kP;
    }

    @Override
    public double getkI() {
        return flushedkI;
    }

    @Override
    public void setkI(double kI) {
        localkI = kI;
    }

    @Override
    public double getkD() {
        return flushedkD;
    }

    @Override
    public void setkD(double kD) {
        localkD = kD;
    }

    @Override
    public void setMaxAbsControlEffortVolts(double maxVolts) {
        localMaxU = maxVolts;
    }

    @Override
    public double getMaxAbsControlEffortVolts() {
        return flushedMaxU;
    }

    @Override
    public void setMaxIntegralAccumulationVolts(double maxAccumulation) {
        localMaxIU = maxAccumulation;
    }

    @Override
    public double getMaxIntegralAccumulationVolts() {
        return flushedMaxIU;
    }

    @Override
    public void setSetpoint(double setpoint) {
        if (localFeedforwardVolts != flushedFeedforward) {
            controller.setFF(localFeedforwardVolts);

            localFeedforwardVolts = flushedFeedforward;
        }

        if (setpoint != localSetpoint) {
            ControlType controlType;

            switch (config.controlType) {
                case POSITION:
                    controlType = ControlType.kPosition;

                    setpoint = setpoint / (2 * Math.PI); // rev uses native revolutions

                    break;
                case CONTINUOUS_ANGLE:
                    controlType = ControlType.kPosition;

                    controller.setPositionPIDWrappingMaxInput(1);
                    controller.setPositionPIDWrappingMinInput(0);

                    controller.setPositionPIDWrappingEnabled(true);

                    setpoint = UtilityMath.normalizeAngleRadians(setpoint) / (2 * Math.PI);

                    break;
                case VELOCITY:
                    controlType = ControlType.kVelocity;

                    setpoint = Units.radiansPerSecondToRotationsPerMinute(setpoint);

                    break;
                case CURRENT:
                    controlType = ControlType.kCurrent;
                    break;
                default:
                    controlType = ControlType.kVoltage; // unreachable anyway
            }

            controller.setReference(setpoint, controlType);
        }

        if (localkP != flushedkP) {
            controller.setP(localkP);

            flushedkP = localkP;
        }

        if (localkI != flushedkI) {
            controller.setI(localkI);

            flushedkI = localkI;
        }

        if (localkD != flushedkD) {
            controller.setD(localkD);

            flushedkD = localkD;
        }

        if (localMaxU != flushedMaxU && localMaxU != 0) {
            controller.setOutputRange(-localMaxU / 12, localMaxU / 12);

            flushedMaxU = localMaxU;
        }

        if (localMaxIU != flushedMaxIU && localMaxIU != 0) {
            controller.setIMaxAccum(localMaxIU / 12, 0);

            flushedMaxIU = localMaxIU;
        }
    }
}
