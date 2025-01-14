/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive.io;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class DriveIOMapleSim extends DriveIOReal {
  private final Pigeon2SimState gyroSimState;
  private final SwerveDriveSimulation driveSim;

  /**
   * Constructs a {@link DriveIOMapleSim} using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves.
   */
  public DriveIOMapleSim(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      MapleSimConfig simConfig,
      SwerveModuleConstants... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    this.gyroSimState = getPigeon2().getSimState();
    var drivetrainSimConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(simConfig.robotMassWithBumpers())
            .withBumperSize(simConfig.bumperLengthX(), simConfig.bumperWidthY())
            .withCustomModuleTranslations(getModuleLocations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    simConfig.driveMotorModel(),
                    simConfig.steerMotorModel(),
                    modules[0].DriveMotorGearRatio,
                    modules[0].SteerMotorGearRatio,
                    Volts.of(modules[0].DriveFrictionVoltage),
                    Volts.of(modules[0].SteerFrictionVoltage),
                    Meters.of(modules[0].WheelRadius),
                    KilogramSquareMeters.of(modules[0].SteerInertia),
                    simConfig.wheelCOF()));
  }

  /**
   * Updates a {@link DriveIOInputs} instance with the latest updates from this {@link
   * DriveIOMapleSim}.
   *
   * @param inputs A {@link DriveIOInputs} instance to update.
   */
  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Update the inputs passed in with the current SwerveDriveState
    inputs.fromSwerveDriveState(getState());

    SimulatedArena.getInstance().simulationPeriodic();
    gyroSimState.setRawYaw(driveSim.getSimulatedDriveTrainPose().getRotation().getMeasure());
    gyroSimState.setAngularVelocityZ(
        RadiansPerSecond.of(
            driveSim.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
  }

  /**
   * Class used to wrap a Talon FX simulation state ({@link TalonFXSimState}) in a {@link
   * SimulatedMotorController}, in order to update the simulation state with the values from a
   * {@link SwerveModuleSimulation}.
   */
  public static class SimulatedTalonFX implements SimulatedMotorController {
    private final TalonFXSimState motorSimState;

    /**
     * Constructs a {@link SimulatedTalonFX} with the provided {@link TalonFX}.
     *
     * @param motor The {@link TalonFX} that this {@link SimulatedTalonFX} will update the
     *     simulation state of.
     */
    public SimulatedTalonFX(TalonFX motor) {
      this.motorSimState = motor.getSimState();
    }

    /**
     * Update the {@link TalonFXSimState} of this {@link SimulatedTalonFX} with the new simulated
     * values.
     *
     * @param mechanismAngle The angle of the mechanism the Talon FX is controlling. This is the
     *     angle reported by the encoder divided by the gear ratio.
     * @param mechanismVelocity The velocity of the mechanism the Talon FX is controlling. This is
     *     the velocity reported by the encoder divided by the gear ratio.
     * @param encoderAngle The angle of the encoder of the Talon FX. This is the direct angle
     *     reported by the encoder.
     * @param encoderVelocity The velocity of the encoder of the Talon FX. This is the direct
     *     velocity reported by the encoder.
     * @return The simulated output voltage of the Talon FX.
     */
    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      // Update motor state with provided values
      motorSimState.setRawRotorPosition(encoderAngle);
      motorSimState.setRotorVelocity(encoderVelocity);
      motorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

      // Get the output voltage of the motor
      return motorSimState.getMotorVoltageMeasure();
    }
  }

  /**
   * Class used to wrap a Talon FX simulation state ({@link TalonFXSimState}) and a remote CANcoder
   * simulation state ({@link CANcoderSimState} in a {@link SimulatedMotorController}, in order to
   * update the simulation states with the values from a {@link SwerveModuleSimulation}.
   */
  public static class SimulatedTalonFXWithCANcoder extends SimulatedTalonFX {
    private final CANcoderSimState encoderSimState;

    /**
     * Constructs a {@link SimulatedTalonFX} with the provided {@link TalonFX} and {@link CANcoder}.
     *
     * @param motor The {@link TalonFX} that this {@link SimulatedTalonFXWithCANcoder} will update
     *     the simulation state of.
     * @param encoder The remote {@link CANcoder} that this {@link SimulatedTalonFXWithCANcoder}
     *     will update the simulation state of.
     */
    public SimulatedTalonFXWithCANcoder(TalonFX motor, CANcoder encoder) {
      super(motor);
      this.encoderSimState = encoder.getSimState();
    }

    /**
     * Update the {@link TalonFXSimState} and {@link CANcoderSimState} of this {@link
     * SimulatedTalonFXWithCANcoder} with the new simulated values.
     *
     * @param mechanismAngle The angle of the mechanism the Talon FX is controlling. This is the
     *     angle reported by the encoder divided by the gear ratio.
     * @param mechanismVelocity The velocity of the mechanism the Talon FX is controlling. This is
     *     the velocity reported by the encoder divided by the gear ratio.
     * @param encoderAngle The angle of the encoder of the Talon FX. This is the direct angle
     *     reported by the encoder.
     * @param encoderVelocity The velocity of the encoder of the Talon FX. This is the direct
     *     velocity reported by the encoder.
     * @return The simulated output voltage of the Talon FX.
     */
    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      encoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      encoderSimState.setRawPosition(mechanismAngle);
      encoderSimState.setVelocity(mechanismVelocity);

      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  public record MapleSimConfig(
      Mass robotMassWithBumpers,
      Distance bumperLengthX,
      Distance bumperWidthY,
      DCMotor driveMotorModel,
      DCMotor steerMotorModel,
      double wheelCOF,
      Translation2d[] moduleLocations) {}
}
