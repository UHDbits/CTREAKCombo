/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class DriveIOMapleSim {
  private final Pigeon2SimState pigeonSim;


  private static class SimSwerveModule {
    private final SwerveModuleSimulation moduleSimulation;

    public SimSwerveModule(SwerveModuleSimulation moduleSimulation, SwerveModule<TalonFX, TalonFX, CANcoder> module) {
      this.moduleSimulation = moduleSimulation;
      moduleSimulation.useDriveMotorController(new SimulatedTalonFX(module.getDriveMotor()));
      moduleSimulation.useSteerMotorController(new SimulatedTalonFXWithCANcoder(module.getSteerMotor(), module.getEncoder()));
    }
  }



  /**
   * Class used to wrap a Talon FX's simulation state ({@link TalonFXSimState}) in a {@link SimulatedMotorController}, in order to update the simulation state with the values from a {@link SwerveModuleSimulation}.
   */
  public static class SimulatedTalonFX implements SimulatedMotorController {
    // Simulation state of a Talon FX
    private final TalonFXSimState motorSimState;

    /**
     * Constructs a {@link SimulatedTalonFX} with the provided {@link TalonFX}.
     *
     * @param motor The {@link TalonFX} that this {@link SimulatedTalonFX} will update the simulation state of.
     */
    public SimulatedTalonFX(TalonFX motor) {
      // Get the simulation state of the Talon FX
      this.motorSimState = motor.getSimState();
    }

    /** Update the {@link TalonFXSimState} of this {@link SimulatedTalonFX} with the new simulated values.
     *
     * @param mechanismAngle The angle of the mechanism the Talon FX is controlling. This is the angle reported by the encoder divided by the gear ratio.
     * @param mechanismVelocity The velocity of the mechanism the Talon FX is controlling. This is the velocity reported by the encoder divided by the gear ratio.
     * @param encoderAngle The angle of the encoder of the Talon FX. This is the direct angle reported by the encoder.
     * @param encoderVelocity The velocity of the encoder of the Talon FX. This is the direct velocity reported by the encoder.
     */
    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      motorSimState.setRawRotorPosition(encoderAngle);
      motorSimState.setRotorVelocity(encoderVelocity);
      motorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

      return motorSimState.getMotorVoltageMeasure();
    }
  }

  public static class SimulatedTalonFXWithCANcoder extends SimulatedTalonFX {
    private final CANcoderSimState encoderSimState;

    public SimulatedTalonFXWithCANcoder(TalonFX motor, CANcoder encoder) {
      super(motor);
      this.encoderSimState = encoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      encoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      encoderSimState.setRawPosition(mechanismAngle);
      encoderSimState.setVelocity(mechanismVelocity);

      return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }
}
