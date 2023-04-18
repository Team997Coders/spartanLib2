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
package org.chsrobotics.lib.models;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import org.chsrobotics.lib.math.UtilityMath;
import org.ejml.simple.SimpleMatrix;

/**
 * Simulation dynamics model of a coaxial swerve drivetrain.
 *
 * @param <N> A Num representing the number of swerve modules.
 */
public class CoaxialSwerveModel<N extends Num> {
    /**
     * Data class containing translations from robot frame for a number of swerve modules.
     *
     * @param xOffsets Vector containing translations in the x dimension, from robot to module, for
     *     each module, in meters. Must be consistently-ordered with other arguments.
     * @param yOffsets Vector containing translations in the y dimension, from robot to module, for
     *     each module, in meters. Must be consistently-ordered with other arguments.
     * @param <J> A Num representing the number of swerve modules.
     */
    public static record CoaxialSwerveModuleOffsets<J extends Num>(
            Vector<J> xOffsets, Vector<J> yOffsets) {}

    /**
     * Data class representing the state of a coaxial swerve drive with J modules.
     *
     * <p>All vectors must be ordered in a consistent way.
     *
     * @param x World-frame X displacement of the robot, in meters.
     * @param y World-frame Y displacement of the robot, in meters.
     * @param vX World-frame X velocity of the robot, in meters per second.
     * @param vY World-frame Y velocity of the robot, in meters per second.
     * @param angle World-frame angle of the robot, in radians.
     * @param angularVelocity World-frame angular velocity of the robot, in radians per second.
     * @param steerAngle Vector holding each module's steer angle, in radians, relative to the
     *     robot.
     * @param steerAngularVelocity Vector holding each module's steer angular velocity, in radians
     *     per second.
     * @param driveAngularAccumulation Vector holding each module's drive angular accumulation (does
     *     not wrap), in radians.
     * @param driveAngularVelocity Vector holding each module's drive angular velocity, in radians
     *     per second.
     * @param <J> A Num representing the number of swerve modules.
     */
    public static record CoaxialSwerveState<J extends Num>(
            double x,
            double y,
            double vX,
            double vY,
            double angle,
            double angularVelocity,
            Vector<J> steerAngle,
            Vector<J> steerAngularVelocity,
            Vector<J> driveAngularAccumulation,
            Vector<J> driveAngularVelocity) {
        private static <K extends Num> CoaxialSwerveState<K> fromStateVec(
                SimpleMatrix stateVec, int numModules) {
            SimpleMatrix moduleSteerAngles = new SimpleMatrix(1, numModules);
            SimpleMatrix moduleSteerAngularVelocities = new SimpleMatrix(1, numModules);
            SimpleMatrix moduleDriveAngularAccumulation = new SimpleMatrix(1, numModules);
            SimpleMatrix moduleDriveAngularVelocities = new SimpleMatrix(1, numModules);

            // convert variable-size module data footer (everything after index 5) to their own
            // vectors
            int index = 6;
            for (int i = 0; i < numModules; i++) {
                moduleSteerAngles.set(
                        0, i, UtilityMath.normalizeAngleRadians(stateVec.get(0, index)));
                index++;

                moduleSteerAngularVelocities.set(0, i, stateVec.get(0, index));
                index++;

                moduleDriveAngularAccumulation.set(0, i, stateVec.get(0, index));
                index++;

                moduleDriveAngularVelocities.set(0, i, stateVec.get(0, index));
                index++;
            }

            return new CoaxialSwerveState<>(
                    stateVec.get(0, 0), // x
                    stateVec.get(0, 1), // y
                    stateVec.get(0, 2), // vx
                    stateVec.get(0, 3), // vy
                    UtilityMath.normalizeAngleRadians(stateVec.get(0, 4)), // angle
                    stateVec.get(0, 5), // angular velocity
                    new Vector<>(moduleSteerAngles),
                    new Vector<>(moduleSteerAngularVelocities),
                    new Vector<>(moduleDriveAngularAccumulation),
                    new Vector<>(moduleDriveAngularVelocities));
        }

        private SimpleMatrix toStateVec(int numStateVars) {
            SimpleMatrix stateAsVector = new SimpleMatrix(1, numStateVars);

            // converting our state object into the kind of Vector the numerical integration class
            // likes

            // set first values of state vector to global state variables
            stateAsVector.set(0, 0, this.x);
            stateAsVector.set(0, 1, this.y);
            stateAsVector.set(0, 2, this.vX);
            stateAsVector.set(0, 3, this.vY);
            stateAsVector.set(0, 4, this.angle);
            stateAsVector.set(0, 5, this.angularVelocity);

            // add a variable-size data footer to the vector, so that we can support arbitrary
            // numbers of modules
            int index = 6;
            for (int i = 0; i < this.steerAngle.getNumCols(); i++) {
                stateAsVector.set(0, index, this.steerAngle.get(0, i));
                index++;

                stateAsVector.set(0, index, this.steerAngularVelocity.get(0, i));
                index++;

                stateAsVector.set(0, index, this.driveAngularAccumulation.get(0, i));
                index++;

                stateAsVector.set(0, index, this.driveAngularVelocity.get(0, i));
                index++;
            }
            return stateAsVector;
        }
    }

    /**
     * Data class representing a control input to a coaxial swerve of J modules.
     *
     * <p>Vectors must be consistently ordered.
     *
     * @param driveInput A vector containing each module's drive input, in volts.
     * @param steerInput A vector containing each module's steer input, in volts.
     * @param <J> A Num representing the number of swerve modules.
     */
    public static record CoaxialSwerveInput<J extends Num>(
            Vector<J> driveInput, Vector<J> steerInput) {
        private SimpleMatrix toInputVec(int numInputVars) {
            SimpleMatrix inputAsVector = new SimpleMatrix(1, numInputVars);
            // loop through each module, changing all inputs to a single vector
            int index = 0;
            for (int i = 0; i < this.driveInput.getNumCols(); i++) {
                inputAsVector.set(0, index, this.steerInput.get(0, i));
                index++;

                inputAsVector.set(0, index, this.driveInput.get(0, i));
                index++;
            }
            return inputAsVector;
        }

        private static <K extends Num> CoaxialSwerveInput<K> fromInputVec(
                SimpleMatrix inputVec, int numModules) {
            Vector<K> steerInputs = new Vector<>(new SimpleMatrix(1, numModules));

            Vector<K> driveInputs = new Vector<>(new SimpleMatrix(1, numModules));

            // TODO implement
            return new CoaxialSwerveInput<>(driveInputs, steerInputs);
        }

        private CoaxialSwerveInput<J> clamp(double driveMaxAbs, double steerMaxAbs) {
            var clDriveInput = driveInput;
            var clSteerInput = steerInput;

            for (int i = 0; i < clDriveInput.getNumCols(); i++) {
                clDriveInput.set(0, i, UtilityMath.clamp(driveMaxAbs, clDriveInput.get(0, i)));

                clSteerInput.set(0, i, UtilityMath.clamp(steerMaxAbs, clSteerInput.get(0, i)));
            }

            return new CoaxialSwerveInput<>(clDriveInput, clSteerInput);
        }
    }

    private final double robotMass;
    private final double robotMoment;

    private final double wheelDriveMoment;
    private final double wheelSteerMoment;

    private final DCMotor driveMotor;
    private final DCMotor steerMotor;

    private final double driveWheelRadius;

    private final CoaxialSwerveModuleOffsets<N> modulePositions;

    private final int numModules;

    private final int numStateVars;

    private final int numInputVars;

    private final double kDriveFrictionTorque;
    private final double kSteerFrictionTorque;

    private class NumInputs extends Num {
        @Override
        public int getNum() {
            return numInputVars;
        }
    }

    private class NumStates extends Num {
        @Override
        public int getNum() {
            return numStateVars;
        }
    }

    private class NumModules extends Num {
        @Override
        public int getNum() {
            return numModules;
        }
    }

    /**
     * Constructs a CoaxialSwerveModel for simulation of a generic coaxial swerve drive base.
     *
     * <p>Note that the ordering of modules in the {@code modulePositions} vector becomes convention
     * for this class's methods.
     *
     * @param robotMass The mass of the robot, in kg.
     * @param robotMoment The moment of inertia of the robot about its z-axis (yaw), in kg m^2.
     * @param wheelDriveMoment The moment of inertia of the driven part of the module and its
     *     geartrain, in kg m^2.
     * @param wheelSteerMoment The moment of inertia of the steered part of the module and its
     *     geartrain, in kg m^2.
     * @param driveMotor DCMotor model of the driving motor. Reductions should already be applied.
     * @param steerMotor DCMotor model of the steering motor. Reductions should already be applied.
     * @param driveWheelRadius Radius of the drive wheel, in meters.
     * @param kDriveFrictionTorque "Fudge factor" frictional torque to apply to the drive action on
     *     a per-module level, in newton meters.
     *     <p>Note that this is intentionally not a coefficient of friction, as there is
     *     robot-mass-independent friction in this system. Users should tune this so this model
     *     behaves like their actual drivetrain, or just take a good guess.
     * @param kSteerFrictionTorque "Fudge factor" frictional torque to apply to the steer action on
     *     a per-module level, in newton meters.
     *     <p>Note that this is intentionally not a coefficient of friction, as there is
     *     robot-mass-independent friction in this system. Users should tune this so this model
     *     behaves like their actual drivetrain, or just take a good guess.
     * @param modulePositions Offsets of module positions relative to the robot frame, in meters.
     */
    public CoaxialSwerveModel(
            double robotMass,
            double robotMoment,
            double wheelDriveMoment,
            double wheelSteerMoment,
            DCMotor driveMotor,
            DCMotor steerMotor,
            double driveWheelRadius,
            double kDriveFrictionTorque,
            double kSteerFrictionTorque,
            CoaxialSwerveModuleOffsets<N> modulePositions) {
        this.robotMass = robotMass;
        this.robotMoment = robotMoment;

        this.wheelDriveMoment = wheelDriveMoment;
        this.wheelSteerMoment = wheelSteerMoment;

        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        this.driveWheelRadius = driveWheelRadius;

        this.kDriveFrictionTorque = kDriveFrictionTorque;
        this.kSteerFrictionTorque = kSteerFrictionTorque;

        this.modulePositions = modulePositions;

        // circumvents type erasure
        numModules = modulePositions.xOffsets.getNumRows();

        numInputVars = numModules * 2;
        // drive input, steer input

        numStateVars = 6 + (4 * numModules);
        // global states: x, y, vx, vy, angle, angular velocity

        // per-module states: steer angle, steer angular velocity,
        // drive angular accumulation, drive angular velocity
    }

    /**
     * Simulates the swerve drivetrain system by with a physics model and numerical integration.
     *
     * @param state The state of the drivetrain before the timestep.
     * @param input Voltage inputs to the drivetrain.
     * @param dtSeconds The timestep to simulate over
     * @return The drivetrain state after the timestep.
     */
    public CoaxialSwerveState<N> simulate(
            CoaxialSwerveState<N> state, CoaxialSwerveInput<N> input, double dtSeconds) {

        SimpleMatrix inputAsVector =
                input.clamp(driveMotor.nominalVoltageVolts, steerMotor.nominalVoltageVolts)
                        .toInputVec(numInputVars);

        // numerical integration to solve the IVP of our system
        Matrix<NumStates, N1> nextState =
                NumericalIntegration.rkdp(
                        this::getStateDerivative,
                        new Matrix<>(state.toStateVec(numStateVars)),
                        new Matrix<>(inputAsVector),
                        dtSeconds);

        return CoaxialSwerveState.fromStateVec(nextState.getStorage(), numModules);
    }

    // returns the derivative of the state with respect to time, given current state and an input
    private Matrix<NumStates, N1> getStateDerivative(
            Matrix<NumStates, N1> xMatrix, Matrix<NumInputs, N1> uMatrix) {

        CoaxialSwerveState<NumModules> state =
                CoaxialSwerveState.fromStateVec(xMatrix.getStorage(), numModules);

        CoaxialSwerveInput<NumModules> input =
                CoaxialSwerveInput.fromInputVec(uMatrix.getStorage(), numModules);

        Vector<NumModules> steerAngularAcceleration = new Vector<>(new SimpleMatrix(1, numModules));
        Vector<NumModules> driveAngularAcceleration = new Vector<>(new SimpleMatrix(1, numModules));

        Translation2d forceSum = new Translation2d();

        double torqueSum = 0;

        // iterate through modules
        for (int i = 0; i < numModules; i++) {
            // calculate torque with an existing DC motor model
            double steerTorque =
                    steerMotor.getTorque(
                            steerMotor.getCurrent(
                                    state.steerAngularVelocity.get(0, i),
                                    input.steerInput.get(0, i)));

            // add the "fudge factor" frictional torque to the system
            // reverse the direction of the angular velocity
            // this is static friction, so it's clamped to not flip the direction of the torque
            steerTorque +=
                    UtilityMath.clamp(
                            Math.abs(steerTorque),
                            -Math.signum(
                                    state.steerAngularVelocity.get(0, i) * kSteerFrictionTorque));

            steerAngularAcceleration.set(0, i, steerTorque / wheelSteerMoment);
            // torque / moment = angular accel

            double driveTorque =
                    driveMotor.getTorque(
                            driveMotor.getCurrent(
                                    state.driveAngularVelocity.get(0, i),
                                    input.driveInput.get(0, i)));

            // perform the same friction calculation as with steer
            driveTorque +=
                    UtilityMath.clamp(
                            Math.abs(driveTorque),
                            -Math.signum(
                                    state.driveAngularVelocity.get(0, i) * kDriveFrictionTorque));

            double forceAlongWheelVec = driveTorque / driveWheelRadius;

            // module angles are in robot frame already, no need to transform our forces
            forceSum =
                    forceSum.plus(
                            new Translation2d(
                                    forceAlongWheelVec,
                                    new Rotation2d(state.steerAngle.get(0, i))));

            // torque = force vector X (cross product) radius vector

            // A X B = |A||B|sin(theta) where theta is angle between vectors
            // this definition isn't TECHNICALLY correct but good enough for us

            Translation2d radiusVector =
                    new Translation2d(
                            -modulePositions.xOffsets.get(0, i),
                            -modulePositions.yOffsets.get(0, i));

            double angleBetween =
                    new Rotation2d(state.steerAngle.get(0, i))
                            .minus(radiusVector.getAngle())
                            .getRadians();

            torqueSum +=
                    radiusVector.getNorm() * Math.abs(forceAlongWheelVec) * Math.sin(angleBetween);

            driveAngularAcceleration.set(0, i, driveTorque / wheelDriveMoment);
            // torque / moment = angular accel
        }

        // rotate forces to world-relative
        forceSum = forceSum.rotateBy(new Rotation2d(-state.angle));

        // set global state derivs to sum of forces/ torques from modules
        double aX = forceSum.getX() / robotMass; // force / mass = accel
        double aY = forceSum.getY() / robotMass;

        double angularAcceleration = torqueSum / robotMoment; // torque / moment = angular accel

        return new Matrix<>(
                new CoaxialSwerveState<>(
                                state.vX,
                                state.vY,
                                aX,
                                aY,
                                state.angularVelocity,
                                angularAcceleration,
                                state.steerAngularVelocity,
                                steerAngularAcceleration,
                                state.driveAngularVelocity,
                                driveAngularAcceleration)
                        .toStateVec(numStateVars));
    }
}
