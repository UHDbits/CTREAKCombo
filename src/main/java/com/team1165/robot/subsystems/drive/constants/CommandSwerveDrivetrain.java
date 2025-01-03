/*
 * File originally made by: The Cheesy Poofs - FRC 254
 * Copyright (c) 2024 Team 254 (https://github.com/Team254)
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive.constants;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * A simple class that allows us to hold the constants from the TunerConstants output by the Phoenix
 * Tuner X Swerve Generator. This allows us to use the TunerConstants without having to modify the
 * file heavily.
 */
public class CommandSwerveDrivetrain {
  SwerveDrivetrainConstants drivetrainConstants;
  SwerveModuleConstants[] moduleConstants;

  /**
   * A simple class that allows us to hold the constants from the TunerConstants output by the
   * Phoenix Tuner X Swerve Generator. This allows us to use the TunerConstants without having to
   * modify the file heavily.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive.
   * @param modules Constants for each specific module.
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
    this.drivetrainConstants = drivetrainConstants;
    this.moduleConstants = modules;
  }

  /**
   * Get the {@link SwerveDrivetrainConstants} of this {@link CommandSwerveDrivetrain}.
   *
   * @return The {@link SwerveDrivetrainConstants} of this {@link CommandSwerveDrivetrain}.
   */
  public SwerveDrivetrainConstants getDrivetrainConstants() {
    return drivetrainConstants;
  }

  /**
   * Get the {@link SwerveModuleConstants} of this {@link CommandSwerveDrivetrain}.
   *
   * @return The {@link SwerveModuleConstants} of this {@link CommandSwerveDrivetrain}. This is
   *     returned in the order of [FrontLeft, FrontRight, BackLeft, BackRight].
   */
  public SwerveModuleConstants[] getModuleConstants() {
    return moduleConstants;
  }
}
