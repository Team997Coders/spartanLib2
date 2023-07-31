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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.util.datalog.DataLog;
import java.util.ArrayList;
import org.chsrobotics.lib.hardware.base.motorController.AbstractSmartMotorController;
import org.chsrobotics.lib.hardware.revRobotics.sparkMax.SparkMaxIncrementalEncoder.SparkMaxIncrementalEncoderConfig;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

/** Wrapper class for a SparkMAX used on the CAN bus. */
public class SpartanSparkMAX extends AbstractSmartMotorController {

    /** Interface for encoders that can be locally used for SparkMAX feedback control. */
    interface SparkMaxRemoteFeedbackDevice {
        /**
         * If you don't know exactly what you're doing, don't use this method. This interface is
         * meant for low-level implementation only.
         *
         * @return REV's low-level type for this feedback device.
         */
        MotorFeedbackSensor getRevSensor();
    }

    public enum MotorType {
        BRUSHLESS,
        BRUSHED;

        private com.revrobotics.CANSparkMaxLowLevel.MotorType asRev() {
            if (this == BRUSHED) return com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
            else return com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
        }
    }

    /**
     * Configuration for a SparkMAX motor controller on the CAN bus.
     *
     * @param motorType Whether this motor is brushless (NEO, NEO 550), or not.
     * @param inverted Whether this motor should be inverted such that a "positive" input induces a
     *     rotation in the opposite direction.
     * @param currentLimitAmps Current, in amps, to limit this motor to. 40 is a reasonable number
     *     for a full-sized motor, 10 or so is recommended for a smaller (550-sized) motor.
     * @param initialIdleMode Whether the motor should start in brake or coast mode.
     */
    public static record SparkMaxConfig(
            MotorType motorType,
            boolean inverted,
            double currentLimitAmps,
            IdleMode initialIdleMode) {}

    private final double hardCurrentLimitModifier = 1.5;
    // multiply the "smart" current limit by this to get the secondary current limit.

    private final SparkMaxConfig config;

    private final CANSparkMax sparkMax;

    private double setVoltage = 0;
    private IdleMode idleMode;

    private boolean logsConstructed = false;

    private Logger<Boolean> stalenessWatchdogTriggeredLogger;

    private Logger<FaultID[]> faultsLogger;
    private Logger<FaultID[]> stickyFaultsLogger;

    private boolean dutyCycleEncoderInUse = false;
    private boolean analogEncoderInUse = false;
    private boolean encPortEncoderInUse = false;
    private boolean dataPortEncoderInUse = false;

    private final int notInUsePeriodMS = 10000;
    private final int inUsePeriodMS = 20;

    private final PeriodicFrame faultsFrame = PeriodicFrame.kStatus0;
    private final PeriodicFrame coreTLMFrame = PeriodicFrame.kStatus1;
    private final PeriodicFrame motorPosFrame = PeriodicFrame.kStatus2;
    private final PeriodicFrame analogFrame = PeriodicFrame.kStatus3;
    private final PeriodicFrame alternateEncoderFrame = PeriodicFrame.kStatus4;
    private final PeriodicFrame dutyCycleEncoderPosFrame = PeriodicFrame.kStatus5;
    private final PeriodicFrame dutyCycleEncoderVelFrame = PeriodicFrame.kStatus6;

    private int stalenessCount = 0;
    private int stalenessThreshold = 0;

    private double previousBusVoltage = 0;

    /**
     * Constructs a SpartanSparkMax.
     *
     * @param canID CAN bus identification index of this motor controller.
     * @param config Configuration to use for this motor controller.
     */
    public SpartanSparkMAX(int canID, SparkMaxConfig config) {
        this.config = config;

        this.sparkMax = new CANSparkMax(canID, config.motorType.asRev());

        configure();

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    /** Returns this motor controller's configuration. */
    public SparkMaxConfig getConfig() {
        return config;
    }

    /**
     * Returns an encoder object cooresponding to an encoder plugged into the side-facing encoder
     * port.
     *
     * @param config Configuration for this encoder.
     * @return Cooresponding encoder object.
     */
    public SparkMaxIncrementalEncoder getEncoderPortEncoder(
            SparkMaxIncrementalEncoder.SparkMaxIncrementalEncoderConfig config) {
        encPortEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(motorPosFrame, inUsePeriodMS);

        return new SparkMaxIncrementalEncoder(
                config, sparkMax.getEncoder(config.type(), (int) config.countsPerRevolution()));
    }

    /**
     * Returns an encoder object cooresponding to the default integrated NEO encoder plugged into
     * the side-facing encoder port.
     *
     * @return Cooresponding encoder object.
     */
    public SparkMaxIncrementalEncoder getDefaultNEOEncoder() {
        return getEncoderPortEncoder(
                new SparkMaxIncrementalEncoderConfig(Type.kHallSensor, false, 42));
    }

    /**
     * Returns an encoder object cooresponding to a quadrature encoder connected to the SparkMAX's
     * upward-facing data port.
     *
     * @param config Configuration for this encoder.
     * @return Cooresponding encoder object.
     */
    public SparkMaxIncrementalEncoder getDataPortEncoder(
            SparkMaxIncrementalEncoder.SparkMaxIncrementalEncoderConfig config) {
        dataPortEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(alternateEncoderFrame, inUsePeriodMS);

        return new SparkMaxIncrementalEncoder(
                config, sparkMax.getAlternateEncoder((int) config.countsPerRevolution()));
    }

    /**
     * Returns an encoder object for a duty cycle (PWM) encoder attached to the SparkMAX's
     * upwards-facing data port.
     *
     * @param config Configuration for this encoder.
     * @return Cooresponding encoder object.
     */
    public SparkMaxDutyCycleAbsoluteEncoder getDataPortAbsoluteDutyCycleEncoder(
            SparkMaxDutyCycleAbsoluteEncoder.SparkMaxDutyCycleAbsoluteEncoderConfig config) {
        dutyCycleEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(dutyCycleEncoderPosFrame, inUsePeriodMS);
        sparkMax.setPeriodicFramePeriod(dutyCycleEncoderVelFrame, inUsePeriodMS);

        return new SparkMaxDutyCycleAbsoluteEncoder(
                sparkMax.getAbsoluteEncoder(
                        com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle),
                config);
    }

    /**
     * Returns an encoder object for an analog encoder connected to the SparkMAX's upwards-facing
     * data port.
     *
     * <p>Analog encoders must have v_max of 5 volts or less, otherwise the higher voltages may not
     * be recognized by the controller, or cause damage.
     *
     * @param config Configuration for this encoder.
     * @return Cooresponding encoder object.
     */
    public SparkMaxAnalogAbsoluteEncoder getDataPortAbsoluteAnalogEncoder(
            SparkMaxAnalogAbsoluteEncoder.SparkMaxAnalogAbsoluteEncoderConfig config) {
        analogEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(analogFrame, inUsePeriodMS);

        return new SparkMaxAnalogAbsoluteEncoder(sparkMax.getAnalog(Mode.kAbsolute), config);
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            super.autoGenerateLogs(log, name, subdirName, publishToNT, recordInLog);

            faultsLogger =
                    new Logger<>(log, name + "/faults", subdirName, publishToNT, recordInLog);

            stickyFaultsLogger =
                    new Logger<>(log, name + "/stickyFaults", subdirName, publishToNT, recordInLog);

            PeriodicCallbackHandler.registerCallback(this::updateLogs);

            logsConstructed = true;
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            stalenessWatchdogTriggeredLogger.update(isStale());

            ArrayList<FaultID> faults = new ArrayList<>();
            ArrayList<FaultID> stickyFaults = new ArrayList<>();

            for (FaultID fault : FaultID.values()) {
                if (sparkMax.getFault(fault)) faults.add(fault);

                if (sparkMax.getStickyFault(fault)) stickyFaults.add(fault);
            }

            faultsLogger.update(faults.toArray(new FaultID[] {}));

            stickyFaultsLogger.update(faults.toArray(new FaultID[] {}));
        }
    }

    @Override
    public void setVoltage(double volts) {
        if (this.setVoltage != volts) {
            sparkMax.setVoltage(volts);
            setVoltage = volts;
        }
    }

    @Override
    public double getSetVoltage() {
        return setVoltage;
    }

    @Override
    public double getAppliedVoltage() {
        return sparkMax.getAppliedOutput() * getBusVoltage();
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        if (idleMode != this.idleMode) {
            sparkMax.setIdleMode(idleMode.asRev());
            this.idleMode = idleMode;
        }
    }

    @Override
    public IdleMode getIdleMode() {
        return idleMode;
    }

    @Override
    public double getBusVoltage() {
        return sparkMax.getBusVoltage();
    }

    @Override
    public double getCurrent() {
        return sparkMax.getOutputCurrent();
    }

    @Override
    public double getMotorTemperature() {
        return sparkMax.getMotorTemperature();
    }

    @Override
    public boolean isStale() {
        return (stalenessCount >= stalenessThreshold);
    }

    /** Resets currently-running staleness counter. */
    public void resetStalenessCount() {
        stalenessCount = 0;
    }

    /**
     * Sets the threshold of unchanged cycles for {@code isStale()} to return true.
     *
     * @param cycles Number of cycles bus voltage must stay the same for this to be considered
     *     potentially stale.
     */
    public void setStalenessThreshold(int cycles) {
        stalenessThreshold = cycles;
    }

    /**
     * Returns a RemotePIDController object to represent a control loop running on the SparkMAX.
     *
     * @param feedbackDevice A feedback device the remote SparkMAX can interface with.
     * @param config Configuration for this controller.
     * @return RemotePIDController native to this controller.
     */
    public SparkMaxRemotePIDController getRemoteController(
            SparkMaxRemoteFeedbackDevice feedbackDevice,
            SparkMaxRemotePIDController.SparkMaxRemoteControllerConfig config) {
        var pid = sparkMax.getPIDController();

        pid.setFeedbackDevice(feedbackDevice.getRevSensor());

        pid.setIAccum(0); // might hold over from past life
        pid.setPositionPIDWrappingEnabled(false);

        return new SparkMaxRemotePIDController(config, pid);
    }

    private void periodic(double dtSeconds) {
        // on sticky fault because it's plausible we miss the fault with the way revlib works
        // sticky faults cleared in configure() so this isn't called every cycle
        if (sparkMax.getStickyFault(FaultID.kHasReset)) configure();

        if (getBusVoltage() == previousBusVoltage) stalenessCount++;
        else resetStalenessCount();

        previousBusVoltage = getBusVoltage();
    }

    private void configure() {
        // Configured values which are not burned to flash are lost on power cycle,
        // we need to be able to reapply our settings at any point.

        // We mess around with status frames like this to reduce canbus utilization, which can be a
        // problem. It can be assumed that nobody cares about what data the various
        // sensors hold until they actually get the object for them, and in the same way that they
        // won't stop caring in the middle of runtime (the frames have no way of returning to slow
        // mode in this implementation).

        sparkMax.clearFaults();

        setIdleMode(config.initialIdleMode);

        sparkMax.setInverted(config.inverted);

        sparkMax.setSmartCurrentLimit((int) config.currentLimitAmps);

        sparkMax.setSecondaryCurrentLimit(config.currentLimitAmps * hardCurrentLimitModifier);

        sparkMax.setIdleMode(idleMode.asRev());

        sparkMax.setPeriodicFramePeriod(faultsFrame, inUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(coreTLMFrame, inUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(
                alternateEncoderFrame, encPortEncoderInUse ? inUsePeriodMS : notInUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(
                analogFrame, analogEncoderInUse ? inUsePeriodMS : notInUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(
                alternateEncoderFrame, dataPortEncoderInUse ? inUsePeriodMS : notInUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(
                dutyCycleEncoderPosFrame, dutyCycleEncoderInUse ? inUsePeriodMS : notInUsePeriodMS);
        sparkMax.setPeriodicFramePeriod(
                dutyCycleEncoderVelFrame, dutyCycleEncoderInUse ? inUsePeriodMS : notInUsePeriodMS);

        // TODO: timeout of zero may or may not work?
        sparkMax.setCANTimeout(0);
    }
}
