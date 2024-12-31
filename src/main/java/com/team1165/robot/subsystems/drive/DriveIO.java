/*
 * File originally made by: The Cheesy Poofs - FRC 254
 * Copyright (c) 2024 Team 254 (https://github.com/Team254)
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * A "hardware" interface/implementation layer for a Swerve Drivetrain powered by the CTRE Swerve
 * Library. Due to the nature of the CTRE Swerve Library, this IO class is limited in replay
 * capabilities.
 */
public class DriveIO {
  /** Class used to store the IO values of a CTRE Swerve Drivetrain. */
  @AutoLog
  class DriveIOInputs extends SwerveDrivetrain.SwerveDriveState {
    DriveIOInputs() {
      this.Pose = new Pose2d();
    }

    public void fromSwerveDriveState(SwerveDrivetrain.SwerveDriveState state) {
      this.Pose = state.Pose;
      this.Speeds = state.Speeds;
      this.ModuleStates = state.ModuleStates;
      this.ModuleTargets = state.ModuleTargets;
      this.ModulePositions = state.ModulePositions;
      this.RawHeading = state.RawHeading;
      this.Timestamp = state.Timestamp;
      this.OdometryPeriod = state.OdometryPeriod;
      this.SuccessfulDaqs = state.SuccessfulDaqs;
      this.FailedDaqs = state.FailedDaqs;
    }
  }
}
