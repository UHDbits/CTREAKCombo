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
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class DriveIOMapleSim {
  private final Pigeon2SimState pigeonSim;
  private final SimSwerveModule[] simModules;

  private static class SimSwerveModule {
    private final SwerveModuleSimulation moduleSimulation;

    public SimSwerveModule()
  }

  /**
   * Class used to wrap a Talon FX's simulation state ({@link TalonFXSimState}) in a {@link SimulatedMotorController}, in order to update the simulation state with the values from a {@link SwerveModuleSimulation}.
   */
  public static class SimulatedTalonFX implements SimulatedMotorController {
    private final TalonFXSimState motorSimState;

    public SimulatedTalonFX(TalonFX motor) {
      this.motorSimState = motor.getSimState();
    }

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
